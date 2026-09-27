"""Complete raw-USD triangle separation and intersection helpers.

Historical anchor34 command-line diagnostic omitted; numerical helpers unchanged.
No mesh edits, hulls, hppfcl or URDF.
3-D Rtree complete AABB candidates; triangle SAT includes coplanar in-plane axes.
Floating near-contact is conservatively unresolved unless a geometric witness exists.
"""
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from rtree import index

TOL = 1e-9  # metres; padded broad phase and normalized SAT intervals


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def sat_gaps(a, b, return_axes=False):
    """Maximum separating interval gap per triangle pair, normalized to metres.

    Two face normals + 9 edge cross-products suffice for noncoplanar triangles;
    six face-normal/own-edge axes additionally handle coplanar triangles.
    Degenerate candidates are reported unresolved by the caller.
    """
    ea = np.roll(a, -1, axis=1) - a
    eb = np.roll(b, -1, axis=1) - b
    na = np.cross(ea[:, 0], ea[:, 1]); nb = np.cross(eb[:, 0], eb[:, 1])
    axes = [na, nb]
    axes += [np.cross(ea[:, i], eb[:, j]) for i in range(3) for j in range(3)]
    axes += [np.cross(na, ea[:, i]) for i in range(3)]
    axes += [np.cross(nb, eb[:, i]) for i in range(3)]
    gap = np.full(len(a), -np.inf)
    best_axis = np.zeros((len(a),3))
    for axis in axes:
        norm = np.linalg.norm(axis, axis=1)
        unit = axis / np.maximum(norm[:, None], 1e-300)
        ap = np.einsum('nvc,nc->nv', a, unit)
        bp = np.einsum('nvc,nc->nv', b, unit)
        g = np.maximum(bp.min(1)-ap.max(1), ap.min(1)-bp.max(1))
        g = np.where(norm > 1e-24, g, -np.inf)
        better = g > gap; best_axis[better] = unit[better]
        gap = np.maximum(gap, g)
    return (gap,best_axis) if return_axes else gap


def barycentric(p, tri):
    # Least squares is independent of SAT; residual and weights are recorded.
    uv, _, _, _ = np.linalg.lstsq((tri[1:]-tri[0]).T, p-tri[0], rcond=None)
    weights = np.array([1-uv.sum(), uv[0], uv[1]])
    return weights, float(np.linalg.norm(weights @ tri-p))


def intersection_witness(a, b):
    na = np.cross(a[1]-a[0], a[2]-a[0]); nb = np.cross(b[1]-b[0], b[2]-b[0])
    if min(np.linalg.norm(na), np.linalg.norm(nb)) < 1e-18:
        return {'status': 'degenerate_candidate_unresolved'}
    na /= np.linalg.norm(na); nb /= np.linalg.norm(nb)
    da = (a-b[0]) @ nb; db = (b-a[0]) @ na
    points = []
    coplanar = np.linalg.norm(np.cross(na, nb)) < 1e-10 and max(abs(da).max(), abs(db).max()) < 1e-11
    if coplanar:
        # Convex clipping in dominant-axis 2D; preserve original 3D points.
        keep = [i for i in range(3) if i != int(np.argmax(abs(na)))]
        poly = list(a.copy())
        bt = b[:, keep]
        cross2 = lambda u, v: float(u[0]*v[1]-u[1]*v[0])
        sign = np.sign(cross2(bt[1]-bt[0], bt[2]-bt[0]))
        for j in range(3):
            old = poly; poly = []
            if not old: break
            p0, p1 = bt[j], bt[(j+1)%3]
            side = lambda p: sign*cross2(p1-p0, p[keep]-p0)
            for k, p in enumerate(old):
                q = old[(k+1)%len(old)]; dp, dq = side(p), side(q)
                if dp >= 0: poly.append(p)
                if (dp < 0) != (dq < 0): poly.append(p + dp/(dp-dq)*(q-p))
        points = poly
    else:
        for tri, distances, other in [(a, da, b), (b, db, a)]:
            for i in range(3):
                j = (i+1)%3; d0, d1 = distances[i], distances[j]
                candidates = []
                if abs(d0) <= 1e-12: candidates.append(tri[i])
                if d0*d1 < 0: candidates.append(tri[i]+d0/(d0-d1)*(tri[j]-tri[i]))
                for p in candidates:
                    w, residual = barycentric(p, other)
                    if w.min() >= -1e-10 and residual <= 1e-11: points.append(p)
    verified = []
    for p in points:
        wa, ra = barycentric(p, a); wb, rb = barycentric(p, b)
        if min(wa.min(), wb.min()) >= -1e-10 and max(ra, rb) <= 1e-11:
            if all(np.linalg.norm(p-np.array(x['point_pelvis_m'])) > 1e-11 for x in verified):
                verified.append({'point_pelvis_m': p.tolist(), 'barycentric_a': wa.tolist(),
                                 'barycentric_b': wb.tolist(), 'residual_m': [ra, rb]})
    return {'status': 'surface_intersection_witness' if verified else 'tolerance_or_degeneracy_unresolved',
            'coplanar': bool(coplanar), 'verified_points': verified}


def self_test():
    a = np.array([[0.,0,0],[1,0,0],[0,1,0]])
    cases = [
        ('noncoplanar_cross', [[.2,.2,-1],[.2,.2,1],[.8,.2,0]], True),
        ('coplanar_overlap', [[.2,.2,0],[.8,.2,0],[.2,.8,0]], True),
        ('coplanar_disjoint_AABB_overlap', [[.7,.7,0],[1.2,.7,0],[.7,1.2,0]], False),
        ('parallel_1um', [[0,0,1e-6],[1,0,1e-6],[0,1,1e-6]], False),
        ('shared_edge', [[0,0,0],[1,0,0],[.5,-1,0]], True),
        ('vertex_touch', [[1,0,0],[2,0,0],[1,1,0]], True),
        ('identical', a.tolist(), True)]
    rows = []
    for name, bv, expected in cases:
        b = np.array(bv, dtype=float)
        gap = float(sat_gaps(a[None], b[None])[0])
        w = intersection_witness(a, b)
        hit = w['status'] == 'surface_intersection_witness'
        assert hit == expected and (gap <= TOL) == expected, (name, gap, w)
        # Symmetry and rigid rotation/translation invariance.
        angle = .417; r = np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
        ar, br = a@r.T+[.13,-.21,.32], b@r.T+[.13,-.21,.32]
        assert (sat_gaps(br[None], ar[None])[0] <= TOL) == expected
        assert (intersection_witness(br, ar)['status'] == 'surface_intersection_witness') == expected
        rows.append({'case': name, 'expected_intersection': expected, 'SAT_gap_m': gap,
                     'witness_confirmed': hit, 'swapped_rigid_transform_check': True})
    return rows
