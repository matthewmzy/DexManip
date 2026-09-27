"""USD-only full robot FK and authored geometry access from extract_usd_robot27.

No import from the old synthesis, Pinocchio/URDF, or collision-filter modules.
The source USD hash is checked before model construction.
"""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
ASSET_DIR=ROOT/'assets/h2_sharpa'


class USDModel:
    def __init__(self,folder=None,*,source_usd=None):
        self.folder=Path(folder or ASSET_DIR)
        self.manifest=json.loads((self.folder/'model.json').read_text())
        source=self.manifest['source_asset']
        self.source_provenance_path=source['path']
        # Historical absolute paths are provenance, never runtime dependencies.
        self.source=Path(source_usd) if source_usd is not None else self.folder/'H2_with_sharpa_flat.usd'
        if hashlib.sha256(self.source.read_bytes()).hexdigest()!=source['sha256']:
            raise ValueError('USD changed; regenerate its model and geometry export')
        self.bodies=self.manifest['bodies'];self.by_path={b['path']:i for i,b in enumerate(self.bodies)}
        self.by_name={b['name']:i for i,b in enumerate(self.bodies)}
        self.base=self.by_path[self.manifest['pelvis_body']]
        self.geometry=self.manifest['geometry'];self.arrays=np.load(self.folder/self.manifest['geometry_archive'])
        self.joints={j['name']:j for j in self.manifest['joints']}
        pending=[j for j in self.joints.values() if j['enabled'] and len(j['body0'])==len(j['body1'])==1]
        self.edges=[];known={self.base}
        while pending:
            progress=False
            for j in pending[:]:
                parent=self.by_path[j['body0'][0]];child=self.by_path[j['body1'][0]]
                if parent not in known:continue
                if child in known:raise ValueError('USD body has multiple enabled parents or a joint cycle')
                if j['type'] not in ['PhysicsFixedJoint','PhysicsRevoluteJoint']:raise ValueError(f"Unsupported joint {j['type']}")
                axis=np.array(j.get('axis',[0,0,0]));k=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                self.edges.append({'name':j['name'],'parent':parent,'child':child,'T0':np.array(j['T0']),
                    'T1_inverse':np.linalg.inv(j['T1']),'K':k,'KK':k@k,'moving':j['type']=='PhysicsRevoluteJoint'})
                known.add(child);pending.remove(j);progress=True
            if not progress:raise ValueError('Unresolved USD joint component; cannot silently assume a fixed body')
        if len(known)!=len(self.bodies):raise ValueError('Some USD rigid bodies are not connected to pelvis')

    def fk(self,joint_positions=None):
        """All body transforms in the pelvis frame; missing joint angles are zero."""
        q={} if joint_positions is None else joint_positions
        unknown=set(q)-set(self.joints)
        if unknown:raise ValueError(f'Unknown USD joints: {unknown}')
        transforms=np.repeat(np.eye(4)[None],len(self.bodies),0)
        for edge in self.edges:
            angle=float(q.get(edge['name'],0));t=np.eye(4)
            if not edge['moving'] and angle!=0:raise ValueError('Nonzero angle supplied for a USD fixed joint')
            if edge['moving']:t[:3,:3]=np.eye(3)+np.sin(angle)*edge['K']+(1-np.cos(angle))*edge['KK']
            transforms[edge['child']]=transforms[edge['parent']]@edge['T0']@t@edge['T1_inverse']
        return transforms

    def mesh(self,index,body_coordinates=True):
        """Authored mesh, retaining original topology; all current asset faces are triangles."""
        g=self.geometry[index]
        if g['type']!='Mesh':raise ValueError('Use exact primitive metadata for nonmesh geometry')
        key=g['array_prefix'];counts=self.arrays[key+'_face_counts'];holes=self.arrays[key+'_holes']
        if np.any(counts!=3) or len(holes):raise ValueError('Nontriangle or holed mesh requires explicit triangulation')
        vertices=self.arrays[key+'_vertices'].copy();faces=self.arrays[key+'_face_indices'].reshape(-1,3).copy()
        if g['orientation']=='leftHanded':faces=faces[:,::-1]
        if body_coordinates:
            t=np.array(g['body_from_geometry']);vertices=vertices@t[:3,:3].T+t[:3,3]
        return vertices,faces

    def geometry_selection(self,representation='visual'):
        """Keep visual and physics-collider representations explicit, never mix certificates.

        Visual uses visible authored non-CollisionAPI meshes; a body with no
        such geometry falls back to its authored collider, recorded per item.
        """
        if representation=='collision':return [(i,'authored_collider') for i,g in enumerate(self.geometry) if g['collision_enabled']]
        if representation!='visual':raise ValueError('representation must be visual or collision')
        selected=[]
        for body in self.bodies:
            indices=[i for i,g in enumerate(self.geometry) if g['body']==body['path'] and not g['collision_api'] and g['visibility']!='invisible']
            if indices:selected.extend((i,'authored_visual') for i in indices)
            else:selected.extend((i,'authored_collider_no_visual') for i,g in enumerate(self.geometry) if g['body']==body['path'] and g['collision_enabled'])
        return selected
