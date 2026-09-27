"""Independent adapter from official cooked89 cache to audit87 interfaces.

Does not alter audit87 or the original USD. Geometry is the convex support set
of the official returned cooked vertices. Returned polygon planes are retained
in the archive, but not silently treated as exactly coplanar with those vertices:
their observed residual is explicitly reported in this adapter's manifest.
"""
import argparse
from itertools import combinations
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial import ConvexHull

from audit_collision87 import ColliderAudit87, check_trajectory_colliders87, GUARD
from check_usd_triangles39 import sha


class CookedColliderAudit89(ColliderAudit87):
    def __init__(self,demo,scope='external_arm',external_clearance=False,cache_dir=None):
        if scope not in ['key','external_arm','right_hand_internal']:
            raise ValueError('Unsupported scope')
        super().__init__(demo,scope='key' if scope=='right_hand_internal' else scope,external_clearance=external_clearance)
        self.cooked_scope=scope;self.cache_dir=Path(cache_dir or demo.model.folder/'cooked89')
        manifest=json.loads((self.cache_dir/'export_manifest.json').read_text())
        result_path=self.cache_dir/'results.json';archive_path=self.cache_dir/'cooked_shapes.npz'
        assert sha(result_path)==manifest['export_results_sha256']
        assert sha(archive_path)==manifest['cooked_archive_sha256']
        assert manifest['USD_sha256']==demo.usd_sha256
        assert manifest['model_sha256']==demo.model_sha256
        assert sha(demo.model.folder/demo.model.manifest['geometry_archive'])==manifest['source_geometry_sha256']
        # This flattened model has exactly one source layer. Resolve its original
        # absolute provenance path to the explicitly selected local USD only.
        layers=manifest['source_layer_hashes']
        if len(layers)!=1 or layers[0]['path']!=demo.model.manifest['source_asset']['path']:
            raise ValueError('Unexpected source layers; review the asset manifest before using this cache')
        assert sha(demo.model.source)==layers[0]['sha256']
        report=json.loads(result_path.read_text());assert report['complete'] and report['process_exit_code']==0
        records={r['geometry_index']:r for r in report['records']};z=np.load(archive_path)
        self.containment_records=[];self.plane_residual_max_m=0.;mesh_count=primitive_count=0
        for shape in self.shapes:
            record=records[shape['gi']];g=demo.model.geometry[shape['gi']]
            assert record['path']==g['path'] and record['body']==g['body'] and record['type']==g['type']
            assert record['collision_attributes']==g['collision_attributes']
            assert np.max(abs(np.asarray(record['body_from_geometry'])-np.asarray(g['body_from_geometry'])))<1e-10
            if shape['type']!='Mesh':
                assert record['shape_attributes']==g['shape_attributes'] and record['status']=='analytic_primitive_no_mesh_cooking_needed'
                self.containment_records.append({'geometry_index':shape['gi'],'type':shape['type'],'exact_same_authored_primitive':True})
                primitive_count+=1;continue
            assert record['status']=='cooked_convexes_exported' and len(record['convexes'])==1
            prefix=record['convexes'][0]['array_prefix'];v=z[prefix+'_vertices_geometry_local'];t=z[prefix+'_body_from_geometry']
            vb=v@t[:3,:3].T+t[:3,3];raw_eq=shape['equations']
            violation=float((vb@raw_eq[:,:3].T+raw_eq[:,3]).max())
            if violation>1e-10:raise ValueError(f'Cooked shape not numerically contained in raw author hull: {g["path"]} {violation}')
            # Full original cooked polygon indices checked; no simplification,
            # repair, shrinking, padding or new filter is introduced here.
            idx=z[prefix+'_indices'];counts=z[prefix+'_polygon_counts'];bases=z[prefix+'_polygon_index_bases'];planes=z[prefix+'_polygon_planes'];edges=[]
            local_max=0.
            for count,base,plane in zip(counts,bases,planes):
                face=idx[base:base+count];local_max=max(local_max,float(abs(v[face]@plane[:3]+plane[3]).max()))
                edges.extend((min(int(a),int(b)),max(int(a),int(b))) for a,b in zip(face,np.roll(face,-1)))
            _,edge_counts=np.unique(np.asarray(edges),axis=0,return_counts=True)
            if not np.all(edge_counts==2):raise ValueError('Cooked polygon boundary is not closed')
            self.plane_residual_max_m=max(self.plane_residual_max_m,local_max)
            hull=ConvexHull(vb)
            self.containment_records.append({'geometry_index':shape['gi'],'type':'Mesh','raw_hull_vertex_count':shape['raw_hull_vertex_count'],
                'cooked_vertex_count':len(vb),'all_cooked_vertices_tested':True,'max_signed_outside_raw_plane_m':violation,
                'containment_numerical_tolerance_m':1e-10,'polygon_vertex_plane_residual_m':local_max,
                'cooked_support_set_contained_in_raw_support_set':True})
            shape.update(vertices=vb,equations=hull.equations,local_transform=np.eye(4),raw_hull_vertex_count=len(vb),
                         cooked_array_prefix=prefix)
            mesh_count+=1
        assert mesh_count==manifest['expected_meshes']==69 and primitive_count==manifest['expected_primitives']==4
        self.cache_manifest=manifest;self.axis_cache={}
        if scope=='right_hand_internal':
            root=demo.model.by_name['right_hand_C_MC'];hand={root}
            for e in demo.model.edges:
                if e['parent'] in hand:hand.add(e['child'])
            disabled={frozenset(demo.model.by_name[n] for n in e['bodies']) for e in self.joint_exclusions}
            self.body_pairs=[];self.pairs=[]
            for i,j in combinations(sorted(hand),2):
                if frozenset((i,j)) in disabled:continue
                ids_a=[k for k,s in enumerate(self.shapes) if s['body_index']==i];ids_b=[k for k,s in enumerate(self.shapes) if s['body_index']==j]
                self.body_pairs.append({'bodies':[demo.model.bodies[i]['name'],demo.model.bodies[j]['name']],
                    'margin_m':0.,'has_authored_colliders_on_both':bool(ids_a and ids_b),'kind':'right_hand_internal'})
                for ia in ids_a:
                    for ib in ids_b:self.pairs.append((ia,ib,0.))
            self.pa=np.array([p[0] for p in self.pairs]);self.pb=np.array([p[1] for p in self.pairs]);self.margin=np.zeros(len(self.pairs))

    def manifest(self):
        result=super().manifest()
        result.update(scope=self.cooked_scope,representation='Convex support sets of official PhysX cooked89 vertices plus original analytic spheres/cylinders',
            cache_directory=str(self.cache_dir),cache_manifest=self.cache_manifest,cooked_adapter_sha256=sha(__file__),
            cooked_polygon_plane_residual_max_m=self.plane_residual_max_m,
            polygon_plane_note='API polygon plane residual is recorded, not silently repaired. Separation/overlap use complete returned cooked vertex support sets.',
            containment_certificate={'mesh_count':69,'primitive_count':4,'records':self.containment_records,
                'raw_gap_guard_m':GUARD,'two_shape_max_containment_tolerance_m':2e-10,
                'inheritance':'A verified raw-hull positive separating gap above required margin+1e-7 remains above required margin after at most2e-10 numerical inclusion allowance; all4 primitives identical.',
                'not_inherited':['raw collision','raw unknown','raw untested frames','different inputs or pair/margin scope']},
            actual_physics_scene_stepped=False,contact_offsets_or_effective_scene_overrides_checked=False)
        if self.cooked_scope=='right_hand_internal':
            result['not_checked']=['robot external/arm pairs','table','intended scissors contacts','visual-only bodies','continuous finger time']
            result['root_rigid_motion_invariance']='Both bodies share the same palm root; per-frame pair nonpenetration is invariant under any common rigid palm transform.'
        return result

    def pair_check(self,*args,**kwargs):
        result=super().pair_check(*args,**kwargs)
        if result['status']=='collision':result['meaning']='Strict common interior of complete official cooked vertex support sets or exact analytic primitives; no dynamics/contact-offset claim.'
        return result
