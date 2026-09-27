"""Bounded reusable USD-authored collider checks, independent of visual meshes.

Mesh convexHull, Sphere and Cylinder support intervals are exact for the
authored representation. Positive axis gaps certify separation. Strict common
interior points certify raw authored convex-body overlap. This does not claim
identity with PhysX cooked hulls, which require effective-stage validation.
"""
import argparse
from itertools import combinations
import json
from pathlib import Path
import time
import numpy as np
from scipy.optimize import linprog, minimize
from scipy.spatial import ConvexHull
from check_usd_triangles39 import sha

RIGHT_NAMES=[f'right_{n}_joint' for n in ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']]
LEFT_NAMES=[n.replace('right_','left_') for n in RIGHT_NAMES]
LEFT_Q=[.6,.18,0.,.6,0.,0.,0.]
GUARD=1e-7


class ColliderAudit87:
    def __init__(self,demo,scope='key',external_clearance=False):
        self.demo=demo;self.model=demo.model;self.scope=scope;self.external_clearance=external_clearance
        self.shapes=[];self.axis_cache={}
        for gi,_ in self.model.geometry_selection('collision'):
            g=self.model.geometry[gi];name=self.model.bodies[self.model.by_path[g['body']]]['name']
            s={'gi':gi,'body':name,'body_index':self.model.by_name[name],'type':g['type'],'path':g['path'],
               'local_transform':np.asarray(g['body_from_geometry'])}
            if g['type']=='Mesh':
                if g['collision_attributes'].get('physics:approximation')!='convexHull':
                    raise ValueError('Unsupported authored mesh approximation: '+g['path'])
                v,_=self.model.mesh(gi);v=np.unique(v,axis=0);h=ConvexHull(v)
                s.update(vertices=v[h.vertices],equations=h.equations,local_transform=np.eye(4),
                         original_vertex_count=len(v),raw_hull_vertex_count=len(h.vertices))
            elif g['type'] in ['Sphere','Cylinder']:
                attrs=g['shape_attributes'];s['radius']=float(attrs['radius'])
                if g['type']=='Cylinder':s.update(height=float(attrs['height']),axis=np.eye(3)['XYZ'.index(attrs['axis'])])
            else:raise ValueError('Unsupported collider primitive '+g['type'])
            self.shapes.append(s)
        moving={self.model.by_name['right_shoulder_pitch_link']}
        for e in self.model.edges:
            if e['parent'] in moving:moving.add(e['child'])
        arm={i for i in moving if self.model.bodies[i]['name'].endswith('_link')}
        palm=self.model.by_name['right_hand_C_MC'];hand=moving-arm
        disabled=set();self.joint_exclusions=[]
        for j in self.model.joints.values():
            if not j['enabled'] or j['collision_enabled']:continue
            if len(j['body0'])==len(j['body1'])==1:
                a,b=self.model.by_path[j['body0'][0]],self.model.by_path[j['body1'][0]]
                disabled.add(frozenset((a,b)));self.joint_exclusions.append({'joint':j['name'],'bodies':[self.model.bodies[a]['name'],self.model.bodies[b]['name']],
                    'basis':'USD physics:collisionEnabled=false on this joint'})
        self.pairs=[];self.body_pairs=[]
        fixed_obstacles={self.model.by_name['torso_link'],self.model.by_name['pelvis']}
        for i,j in combinations(range(len(self.model.bodies)),2):
            if frozenset((i,j)) in disabled:continue
            external=(i in moving)!=(j in moving)
            arm_internal=i in arm and j in arm
            arm_hand=(i in arm and j in hand) or (j in arm and i in hand)
            key=(i in fixed_obstacles and j in arm|{palm}) or (j in fixed_obstacles and i in arm|{palm}) or arm_internal
            use=key if scope=='key' else external or arm_internal or arm_hand
            if scope not in ['key','external_arm']:raise ValueError('scope must be key or external_arm')
            if not use:continue
            names=[self.model.bodies[i]['name'],self.model.bodies[j]['name']]
            structural=set(names) in [set(['torso_link','right_shoulder_roll_link']),set(['torso_link','left_shoulder_roll_link'])]
            margin=.01 if external_clearance and external and not structural else 0.
            ids_a=[k for k,s in enumerate(self.shapes) if s['body_index']==i]
            ids_b=[k for k,s in enumerate(self.shapes) if s['body_index']==j]
            self.body_pairs.append({'bodies':names,'margin_m':margin,'has_authored_colliders_on_both':bool(ids_a and ids_b),
                'kind':'external' if external else 'arm_internal' if arm_internal else 'arm_hand'})
            for ia in ids_a:
                for ib in ids_b:self.pairs.append((ia,ib,margin))
        self.pairs.sort(key=lambda row:0 if {self.shapes[row[0]]['body'],self.shapes[row[1]]['body']}=={'torso_link','right_shoulder_roll_link'} else 1)
        self.pa=np.array([p[0] for p in self.pairs]);self.pb=np.array([p[1] for p in self.pairs]);self.margin=np.array([p[2] for p in self.pairs])

    def manifest(self):
        return {'scope':self.scope,'body_pairs':self.body_pairs,'shape_pair_count':len(self.pairs),
            'joint_exclusions':self.joint_exclusions,'left_joint_names':LEFT_NAMES,'left_q_rad':LEFT_Q,
            'other_fixed_joints':'zero; must be matched in deployed scene','external_1cm_enabled':self.external_clearance,
            'new_internal_pair_rule':'zero-margin authored collider nonpenetration only; no new clearance exceptions',
            'numerical_guard_m':GUARD,'collider_count':len(self.shapes),'USD_sha256':self.demo.usd_sha256,
            'model_sha256':self.demo.model_sha256,'script_sha256':sha(__file__),
            'representation':'raw authored USD collision convexHull/sphere/cylinder, not original visual surface and not PhysX cooked geometry',
            'not_checked':['robot/table','hand/object intended contact','right finger/finger pairs','fixed robot internal pairs',
                           'visual surfaces of bodies with no collider','continuous time or DR','PhysX cooked shape equivalence'],
            'no_arbitrary_body_filter_added':True}

    def world(self,q,fingers):
        cfg=dict(zip(LEFT_NAMES,LEFT_Q));cfg.update(zip(RIGHT_NAMES,q));cfg.update(zip(self.demo.finger_names,fingers))
        fk=self.model.fk(cfg);world=[]
        for s in self.shapes:
            t=fk[s['body_index']]@s['local_transform'];r,p=t[:3,:3],t[:3,3]
            if s['type']=='Mesh':
                vertices=s['vertices']@r.T+p;eq=s['equations'];normals=eq[:,:3]@r.T
                w={'type':'Mesh','vertices':vertices,'equations':np.column_stack((normals,eq[:,3]-normals@p)),
                   'r':r,'center':vertices.mean(0)}
            else:
                w={'type':s['type'],'center':p,'radius':s['radius'],'r':r}
                if s['type']=='Cylinder':w.update(axis=r@s['axis'],height=s['height'])
            low,high=self.support(w,np.eye(3));w['box']=np.stack((low,high));world.append(w)
        return world

    @staticmethod
    def support(w,axes):
        if w['type']=='Mesh':
            values=w['vertices']@axes.T;return values.min(0),values.max(0)
        mid=w['center']@axes.T;length=np.linalg.norm(axes,axis=1)
        if w['type']=='Sphere':radius=w['radius']*length
        else:
            axial=axes@w['axis'];radius=.5*w['height']*np.abs(axial)+w['radius']*np.sqrt(np.maximum(length**2-axial**2,0))
        return mid-radius,mid+radius

    @staticmethod
    def signed_interior(w,point):
        if w['type']=='Mesh':return float(-(w['equations'][:,:3]@point+w['equations'][:,3]).max())
        delta=point-w['center']
        if w['type']=='Sphere':return float(w['radius']-np.linalg.norm(delta))
        z=delta@w['axis'];return float(min(.5*w['height']-abs(z),w['radius']-np.linalg.norm(delta-z*w['axis'])))

    def pair_check(self,a,b,pair_index,margin,deadline):
        axes=[np.eye(3)]
        if pair_index in self.axis_cache:axes.insert(0,self.axis_cache[pair_index][None]@a['r'].T)
        between=b['center']-a['center']
        if np.linalg.norm(between)>1e-12:axes.append((between/np.linalg.norm(between))[None])
        aa=np.concatenate(axes);al,ah=self.support(a,aa);bl,bh=self.support(b,aa)
        gaps=np.maximum(bl-ah,al-bh);best=int(gaps.argmax());gap=float(gaps[best]);axis=aa[best]
        if gap>margin+GUARD:return {'status':'pass','gap_lower_bound_m':gap,'axis_pelvis':axis.tolist()}
        if time.monotonic()>=deadline:return {'status':'unknown','reason':'time_bound','gap_lower_bound_m':gap}
        # Linear separation proposal is cheap in three dimensions. Verification
        # uses complete support intervals, never the LP's tolerance alone.
        if a['type']==b['type']=='Mesh':
            av,bv=a['vertices'],b['vertices']
            mat=np.concatenate([np.column_stack((av,np.ones(len(av)),np.ones(len(av)))),
                                np.column_stack((-bv,-np.ones(len(bv)),np.ones(len(bv))))])
            sol=linprog([0.,0.,0.,0.,-1.],A_ub=mat,b_ub=np.zeros(len(mat)),bounds=[(-1.,1.)]*3+[(None,None),(None,1.)],
                        method='highs',options={'time_limit':max(.01,min(.3,deadline-time.monotonic()))})
            if sol.success and np.linalg.norm(sol.x[:3])>1e-12:
                ax=sol.x[:3]/np.linalg.norm(sol.x[:3]);al,ah=self.support(a,ax[None]);bl,bh=self.support(b,ax[None])
                gg=float(max(bl[0]-ah[0],al[0]-bh[0]))
                if gg>gap:gap,axis=gg,ax
                if gap>margin+GUARD:
                    self.axis_cache[pair_index]=axis@a['r'];return {'status':'pass','gap_lower_bound_m':gap,'axis_pelvis':axis.tolist()}
            eq=np.concatenate([a['equations'],b['equations']])
            sol=linprog([0.,0.,0.,-1.],A_ub=np.column_stack((eq[:,:3],np.ones(len(eq)))),b_ub=-eq[:,3],
                        bounds=[(None,None)]*4,method='highs',options={'time_limit':max(.01,min(.3,deadline-time.monotonic()))})
            point=sol.x[:3] if sol.success else None
        else:
            def objective(x):return -x[3]
            def constraint(x):return np.array([self.signed_interior(a,x[:3])-x[3],self.signed_interior(b,x[:3])-x[3]])
            sol=minimize(objective,np.r_[(a['center']+b['center'])/2,-.01],method='SLSQP',constraints=[{'type':'ineq','fun':constraint}],
                         options={'maxiter':40,'ftol':1e-11})
            point=sol.x[:3] if sol.success else None
        if point is not None:
            slack=min(self.signed_interior(a,point),self.signed_interior(b,point))
            if slack>GUARD:return {'status':'collision','meaning':'strict overlap of raw USD-authored convex bodies; cooked PhysX pending',
                'common_point_pelvis_m':point.tolist(),'verified_common_interior_radius_m':slack,'gap_lower_bound_m':gap}
        return {'status':'unknown','reason':'no sufficient required-gap certificate and no verified strict common interior','gap_lower_bound_m':gap,
                'axis_pelvis':axis.tolist(),'margin_m':margin}

    def check_pose(self,q,fingers,*,deadline,first_failure=True):
        world=self.world(q,fingers);boxes=np.stack([w['box'] for w in world]);gap=np.linalg.norm(np.maximum(np.maximum(
            boxes[self.pa,0]-boxes[self.pb,1],boxes[self.pb,0]-boxes[self.pa,1]),0),axis=1)
        pending=np.flatnonzero(gap<=self.margin+GUARD);pass_count=len(self.pairs)-len(pending);issues=[];tightest=None
        for pi in pending:
            if time.monotonic()>=deadline:
                issues.append({'status':'unknown','reason':'time_bound','remaining_pairs':len(pending)});break
            ia,ib,margin=self.pairs[pi];r=self.pair_check(world[ia],world[ib],int(pi),margin,deadline)
            r.update(pair_index=int(pi),bodies=[self.shapes[ia]['body'],self.shapes[ib]['body']],
                     geometry_indices=[self.shapes[ia]['gi'],self.shapes[ib]['gi']],margin_m=margin)
            if tightest is None or r.get('gap_lower_bound_m',np.inf)<tightest.get('gap_lower_bound_m',np.inf):tightest=r
            if r['status']=='pass':pass_count+=1
            else:
                issues.append(r)
                if first_failure:break
        status='collision' if any(r['status']=='collision' for r in issues) else 'unknown' if issues else 'pass'
        return {'status':status,'passed_shape_pairs':pass_count,'shape_pair_count':len(self.pairs),'issues':issues,'tightest_checked_pair':tightest,
                'complete_pair_check':pass_count==len(self.pairs),'hand_FK':None}


def check_trajectory_colliders87(demo,q,fingers,*,max_seconds=60.,scope='key',audit=None,external_clearance=False):
    """q (T,7) or (N,T,7); fingers (T,22) or (N,T,22). No q changes.

    Returns finite-frame raw-authored-collider pass/unknown/collision. A pass is
    ONLY the exact pair manifest and supplied samples; no cooked/visual claim.
    Passing ``audit=ColliderAudit87(demo,scope)`` reuses geometry and axis seeds.
    """
    started=time.monotonic();deadline=started+min(float(max_seconds),120.)
    audit=audit or ColliderAudit87(demo,scope,external_clearance)
    q=np.asarray(q);q=q[None] if q.ndim==2 else q
    fingers=np.asarray(fingers);fingers=np.broadcast_to(fingers,(len(q),)+fingers.shape) if fingers.ndim==2 else fingers
    assert q.shape[-1]==7 and fingers.shape[:2]==q.shape[:2] and fingers.shape[-1]==len(demo.finger_names)
    codes=np.zeros(q.shape[:2],np.int8);issues=[];count=0
    for member in range(len(q)):
        for frame in range(q.shape[1]):
            if time.monotonic()>=deadline:break
            r=audit.check_pose(q[member,frame],fingers[member,frame],deadline=deadline)
            codes[member,frame]={'pass':1,'unknown':2,'collision':3}[r['status']];count+=1
            if r['status']!='pass':
                issues.append({'member':member,'frame':frame,**r})
        if time.monotonic()>=deadline:break
    status='collision' if (codes==3).any() else 'unknown' if (codes!=1).any() else 'pass'
    return {'status':status,'scope':audit.manifest(),'frame_status_codes':codes.tolist(),'status_codes':{'0':'unchecked','1':'pass','2':'unknown','3':'raw_authored_collision'},
        'checked_frames':count,'total_frames':codes.size,'passed_frames':int((codes==1).sum()),'unknown_frames':int((codes==2).sum()),
        'collision_frames':int((codes==3).sum()),'unchecked_frames':int((codes==0).sum()),'issues':issues,
        'passed_members':np.flatnonzero((codes==1).all(1)).tolist(),'elapsed_s':time.monotonic()-started,
        'actual_PhysX_cooked_validation':False,'full_task_pass':False,'training_ready':False}
