"""Full authored USD visual geometry in body coordinates; no URDF dependency.

Builds complete source meshes and conservative NumPy projection certificates.
Optional native triangle BVHs are diagnostic only: the installed hpp-fcl query
path has crashed in sustained runs. Do not use its successful count as admission.
A positive surface distance alone also cannot exclude solid containment.
"""
from itertools import combinations
import numpy as np
import trimesh
from usd_model27 import USDModel


def triangle_bvh(vertices, faces):
    import hppfcl as fcl
    vv=fcl.StdVec_Vec3f(); vv.extend(vertices)
    ff=fcl.StdVec_Triangle(); ff.extend([fcl.Triangle(*map(int,x)) for x in faces])
    model=fcl.BVHModelOBBRSS()
    if model.beginModel(len(faces),len(vertices))!=0: raise RuntimeError('BVH begin failed')
    if model.addSubModel(vv,ff)!=0: raise RuntimeError('BVH add failed')
    if model.endModel()!=0: raise RuntimeError('BVH end failed')
    return model


class USDVisualGeometry:
    def __init__(self, model=None, native_bvh=False):
        self.model=model or USDModel(); self.meshes={}; self.bvhs={}; self.hull_points={};self.hull_normals={}; self.records=[]
        groups={}
        for index,origin in self.model.geometry_selection('visual'):
            g=self.model.geometry[index]
            if g['type']!='Mesh': raise ValueError('Visual primitive requires explicit exact implementation')
            name=self.model.bodies[self.model.by_path[g['body']]]['name']
            v,f=self.model.mesh(index); groups.setdefault(name,[]).append((index,origin,v,f))
        for name,parts in groups.items():
            vertices=[]; faces=[]; offset=0
            for index,origin,v,f in parts:
                vertices.append(v); faces.append(f+offset); offset+=len(v)
            v=np.concatenate(vertices); f=np.concatenate(faces)
            # Exact duplicate merging only: no mesh simplification, convexification,
            # vertex rounding, scale change, or topology repair.
            v,inv=np.unique(v,axis=0,return_inverse=True); f=inv[f]
            mesh=trimesh.Trimesh(v,f,process=False)
            self.meshes[name]=mesh
            if native_bvh:self.bvhs[name]=triangle_bvh(v,f)
            hull=mesh.convex_hull
            self.hull_points[name]=np.asarray(hull.vertices).copy()
            # Any axis with disjoint projected intervals is a valid positive
            # certificate. Coalescing directions cannot create a false positive:
            # intervals are recomputed on the complete hull point set below.
            normals=np.unique(np.round(hull.face_normals,6),axis=0)
            self.hull_normals[name]=normals/np.linalg.norm(normals,axis=1)[:,None]
            self.records.append({'body':name,'geometries':[self.model.geometry[p[0]]['path'] for p in parts],
                'geometry_origin':[p[1] for p in parts], 'vertices':len(v),'triangles':len(f),
                'watertight':bool(mesh.is_watertight),'winding_consistent':bool(mesh.is_winding_consistent),
                'bounds_body':mesh.bounds.tolist()})

    def transforms(self,q=None):
        t=self.model.fk(q)
        return {n:t[self.model.by_name[n]] for n in self.meshes}

    def world_bounds(self,transforms):
        bounds={}
        for n,m in self.meshes.items():
            T=transforms[n]; center=m.bounds.mean(0); radius=np.diff(m.bounds,axis=0)[0]/2
            c=T[:3,:3]@center+T[:3,3]; r=np.abs(T[:3,:3])@radius
            bounds[n]=np.stack([c-r,c+r])
        return bounds

    @staticmethod
    def box_distance(a,b):
        return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0)))

    def distance(self,a,b,transforms):
        """Unqualified native diagnostic; not for sustained search/admission."""
        import hppfcl as fcl
        if not self.bvhs:raise RuntimeError('Native triangle queries disabled for this instance')
        ta=transforms[a]; tb=transforms[b]
        fa=fcl.Transform3f(ta[:3,:3],ta[:3,3]);fb=fcl.Transform3f(tb[:3,:3],tb[:3,3])
        # The installed hpp-fcl binding corrupts later Python state on some
        # intersecting BVH distance queries. Resolve overlap first, and do not
        # request unneeded nearest-point data. No penetration depth is queried.
        cq=fcl.CollisionRequest();cq.enable_contact=False;cr=fcl.CollisionResult()
        fcl.collide(self.bvhs[a],fa,self.bvhs[b],fb,cq,cr)
        if cr.isCollision():return 0.
        req=fcl.DistanceRequest(); req.enable_nearest_points=False
        res=fcl.DistanceResult()
        d=fcl.distance(self.bvhs[a],fa,self.bvhs[b],fb,req,res)
        return float(d)

    def hull_separated(self,a,b,transforms,margin):
        """Conservative positive certificate; False means unresolved, not mesh collision.

        Pure NumPy projected intervals on both hulls' face directions. Missing
        edge-cross axes can only leave more pairs unresolved. Includes nested
        shapes correctly: containment cannot have disjoint intervals. Native
        hpp-fcl is deliberately absent from the workspace certificate path.
        """
        ta=transforms[a];tb=transforms[b]
        # Work in frame a to avoid large-world-coordinate cancellation.
        R=ta[:3,:3].T@tb[:3,:3];p=ta[:3,:3].T@(tb[:3,3]-ta[:3,3])
        av=self.hull_points[a];bv=self.hull_points[b]@R.T+p
        axes=np.concatenate([self.hull_normals[a],self.hull_normals[b]@R.T])
        # Stop at the first witness axis; untested axes cannot invalidate an
        # already established gap. This avoids large dense temporary matrices.
        for start in range(0,len(axes),32):
            normal=axes[start:start+32];ap=av@normal.T;bp=bv@normal.T
            gap=np.maximum(bp.min(0)-ap.max(0),ap.min(0)-bp.max(0))
            if np.any(gap>margin+1e-10):return True
        return False

    def graph_pair_inventory(self):
        """Inventory only; direct neighbors are structural, not arbitrary filter lists.

        No old hand/arm collision whitelist is imported. All other body pairs
        remain visible, including finger pairs and torso/shoulder-roll pairs.
        """
        connected={frozenset((self.model.bodies[e['parent']]['name'],self.model.bodies[e['child']]['name'])):e
            for e in self.model.edges}
        out=[]
        for a,b in combinations(self.meshes,2):
            edge=connected.get(frozenset((a,b)))
            classification='nonadjacent'
            if edge: classification='joint_adjacent' if edge['moving'] else 'rigidly_connected'
            if set((a,b)) in [set(('torso_link','right_shoulder_roll_link')),set(('torso_link','left_shoulder_roll_link'))]:
                classification='user_structural_nonpenetration_exception'
            out.append({'a':a,'b':b,'classification':classification,'joint':None if edge is None else edge['name']})
        return out
