"""Render a saved complete SI pose using original USD visible triangles.

Usage: python render_usd_pose.py --center <center.json> --out <new.png>
No physics step or hardware access. Output is only a geometry preview.
"""
import argparse
import json
import os
from pathlib import Path
import sys

os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import numpy as np

from si_core import ROOT


def render(center_path,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    sys.path.insert(0,str(ROOT/'vendor/usd_geometry'))
    from usd_model27 import USDModel
    from usd_geometry27 import USDVisualGeometry
    output=Path(output)
    if output.exists():
        raise FileExistsError(output)
    q=json.loads(Path(center_path).read_text())['joint_positions_rad']
    model=USDModel();geometry=USDVisualGeometry(model,native_bvh=False)
    transforms=geometry.transforms(q)
    fig=plt.figure(figsize=(11,8))
    light=np.array([.6,-.3,.74]);light/=np.linalg.norm(light)
    for k,(azimuth,title) in enumerate(((0,'Front'),(-90,'Side'))):
        ax=fig.add_subplot(1,2,k+1,projection='3d')
        for name,mesh in geometry.meshes.items():
            transform=transforms[name]
            vertices=np.asarray(mesh.vertices)@transform[:3,:3].T+transform[:3,3]
            tri=vertices[np.asarray(mesh.faces)]
            normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
            normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-15)
            moving=any(part in name for part in ('shoulder','elbow','wrist','hand','thumb','index','middle','ring','pinky'))
            color=np.array([.16,.52,.82] if moving else [.65,.68,.71])
            shade=.48+.52*np.abs(normals@light)
            ax.add_collection3d(Poly3DCollection(tri,facecolors=np.c_[shade[:,None]*color, np.ones(len(tri))],
                linewidths=0,edgecolors='none',rasterized=True))
        ax.set_xlim(-.4,.65);ax.set_ylim(-.8,.8);ax.set_zlim(-1.05,.95)
        ax.set_box_aspect([1.05,1.6,2.]);ax.set_proj_type('ortho');ax.view_init(elev=8,azim=azimuth)
        ax.set_title(title);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.set_zlabel('z, pelvis frame (m)')
        if k==0:
            ax.set_xticks([]);ax.set_xlabel('')
        else:
            ax.set_yticks([]);ax.set_ylabel('')
        ax.grid(False);ax.tick_params(labelsize=7)
    fig.suptitle('SI candidate from the project USD | arms/hands blue; fixed body grey')
    fig.text(.5,.025,'Empty workspace; table removed. Geometry preview only; not a measured or executed robot pose.',ha='center',fontsize=9)
    fig.subplots_adjust(left=.02,right=.98,bottom=.08,top=.92,wspace=.06)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=150);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--center',required=True);parser.add_argument('--out',required=True)
    args=parser.parse_args();render(args.center,args.out)
