"""Offline full-state USD geometry checks for bilateral arm/hand SI.

Reuses the repository's verified cooked89 cache and USD FK. No Isaac startup,
robot connection, URDF, mesh reduction, or implicit fixed-body pose is used.
Missing-collider bodies have visible meshes: their relevant pairs additionally
require separation of complete authored visual hulls or original triangles with
solid-containment exclusion. Hull overlap alone is never a mesh-collision claim.
"""
from __future__ import annotations

from itertools import combinations
import hashlib
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import trimesh
from rtree import index as rtree_index

from si_core import CHANNELS, ROOT

PREFLIGHT_SCRIPTS = ROOT / "vendor/usd_geometry"
if str(PREFLIGHT_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(PREFLIGHT_SCRIPTS))

from cooked_collision89 import CookedColliderAudit89
from audit_collision87 import GUARD
from usd_geometry27 import USDVisualGeometry
from usd_model27 import USDModel
from check_usd_triangles39 import sat_gaps


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class SIUSDGeometry(CookedColliderAudit89):
    """Check a complete 75-revolute-joint state, in radians and pelvis frame.

    ``check_pose(q, deadline=...)`` accepts an absolute ``time.monotonic()``
    deadline, defaulting to ten seconds. It returns pass/unknown/collision/limits.
    Only USD joint collision-disable relationships are excluded. Fixed
    head/waist/legs remain obstacles; their mutual static pairs are outside SI.
    A pass applies to the stated mixed cooked/visual coverage at this pose,
    not to dynamics, all visual surfaces, or continuous motion between poses.
    """

    def __init__(self, model_folder=None, cache_dir=None):
        model = USDModel(model_folder)
        demo = SimpleNamespace(model=model, usd_sha256=model.manifest["source_asset"]["sha256"],
                               model_sha256=_sha(model.folder / "model.json"))
        super().__init__(demo, scope="external_arm", cache_dir=cache_dir)
        self.movable_joint_names = tuple(n for names in CHANNELS.values() for n in names)
        self.revolute_joint_names = tuple(j["name"] for j in self.model.joints.values()
                                         if j["enabled"] and j["type"] == "PhysicsRevoluteJoint")
        if len(self.movable_joint_names) != 58 or not set(self.movable_joint_names) <= set(self.revolute_joint_names):
            raise ValueError("SI control channels do not match the USD's 14 arm and 44 hand joints")
        self.fixed_joint_names = tuple(n for n in self.revolute_joint_names if n not in self.movable_joint_names)
        moving = {model.by_name[f"{s}_shoulder_pitch_link"] for s in ("left", "right")}
        for edge in model.edges:
            if edge["parent"] in moving:
                moving.add(edge["child"])
        self.moving_body_names = tuple(model.bodies[i]["name"] for i in sorted(moving))
        disabled = {frozenset(model.by_name[n] for n in e["bodies"]) for e in self.joint_exclusions}
        by_body = {i: [k for k, s in enumerate(self.shapes) if s["body_index"] == i]
                   for i in range(len(model.bodies))}
        self.visual = USDVisualGeometry(model, native_bvh=False)
        self.no_collider_bodies = tuple(model.bodies[i]["name"] for i, ids in by_body.items() if not ids)
        self.no_geometry_bodies = tuple(n for n in self.no_collider_bodies if n not in self.visual.meshes)
        if self.no_geometry_bodies:
            raise ValueError(f"USD bodies have neither enabled colliders nor visible geometry: {self.no_geometry_bodies}")
        self.pairs, self.body_pairs, self.visual_pairs = [], [], []
        for i, j in combinations(range(len(model.bodies)), 2):
            if not (i in moving or j in moving) or frozenset((i, j)) in disabled:
                continue
            names = (model.bodies[i]["name"], model.bodies[j]["name"])
            has_colliders = bool(by_body[i] and by_body[j])
            self.body_pairs.append(dict(bodies=list(names), margin_m=0.,
                                        has_authored_colliders_on_both=has_colliders,
                                        kind="both_moving" if i in moving and j in moving else "moving_vs_fixed"))
            if has_colliders:
                self.pairs.extend((a, b, 0.) for a in by_body[i] for b in by_body[j])
            else:
                self.visual_pairs.append(names)
        self.pa = np.array([p[0] for p in self.pairs], dtype=int)
        self.pb = np.array([p[1] for p in self.pairs], dtype=int)
        self.margin = np.zeros(len(self.pairs))
        self.axis_cache = {}
        self._triangle_assets = {}
        self._surface_certificates = {}
        self._relative_paths = {}

    def manifest(self):
        return dict(
            scope="both_arms_both_hands_against_each_other_and_fixed_body",
            source_usd=str(self.model.source), USD_sha256=self.demo.usd_sha256,
            model_sha256=self.demo.model_sha256, cooked_cache_manifest=self.cache_manifest,
            adapter_sha256=_sha(__file__), geometry_archive_sha256=_sha(self.model.folder / self.model.manifest["geometry_archive"]),
            reused_source_sha256={name: _sha(PREFLIGHT_SCRIPTS / name) for name in
                                 ("usd_model27.py", "usd_geometry27.py", "audit_collision87.py", "cooked_collision89.py", "check_usd_triangles39.py")},
            movable_joint_names=list(self.movable_joint_names), fixed_joint_names=list(self.fixed_joint_names),
            fixed_joint_values="Supplied explicitly in each complete q state; no fixed-pose default",
            moving_body_names=list(self.moving_body_names), body_pairs=self.body_pairs,
            cooked_shape_count=len(self.shapes), cooked_shape_pair_count=len(self.pairs),
            visible_bodies_without_collider=list(self.no_collider_bodies),
            missing_all_geometry_bodies=list(self.no_geometry_bodies),
            supplemental_visual_pair_count=len(self.visual_pairs),
            supplemental_visual_pairs=[list(p) for p in self.visual_pairs],
            joint_exclusions=self.joint_exclusions,
            exclusion_basis="Only enabled USD joints with physics:collisionEnabled=false; no arbitrary hand or torso whitelist",
            nonadjacent_finger_pairs_included=True, numerical_guard_m=GUARD,
            cooked_representation="Complete official PhysX cooked89 vertex support sets plus authored analytic sphere/cylinder primitives",
            cooked_polygon_plane_residual_max_m=self.plane_residual_max_m,
            supplemental_representation="Complete visible mesh hull separation; unresolved hulls additionally use complete original triangle AABBs/SAT and component-wise exclusion of solid containment",
            supplemental_cache="Exact (unrounded) values of USD joints on the relative body path; common upstream rigid motion cancels",
            supplemental_reason="15 visible USD bodies have no enabled collider; they must not disappear from the SI obstacle check",
            mesh_simplification_applied=False, actual_physics_scene_stepped=False,
            not_checked=["continuous time between supplied poses", "dynamics and tracking error", "external obstacles (table removed by user)",
                         "fixed-body pairs with neither endpoint in an arm/hand", "all visible surfaces on pairs already covered by cooked geometry",
                         "PhysX contact offsets or effective scene overrides"])

    def _triangles(self, name):
        if name not in self._triangle_assets:
            mesh = self.visual.meshes[name]
            tri = np.asarray(mesh.triangles)
            low, high = tri.min(1), tri.max(1)
            prop = rtree_index.Property()
            prop.dimension = 3
            # Avoid the ctypes callback used by Rtree's stream constructor: a
            # fresh environment intermittently aborted inside that native path.
            # Explicit insertion indexes the same complete, padded AABBs; it
            # changes neither triangle coverage nor the separation test.
            tree = rtree_index.Index(properties=prop)
            for i in range(len(tri)):
                tree.insert(i, tuple(np.r_[low[i] - GUARD, high[i] + GUARD]))
            components = []
            for faces in trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), engine="scipy"):
                raw_ids = np.asarray(mesh.faces)[faces]
                ids, inverse = np.unique(raw_ids, return_inverse=True)
                # Submeshes only index original vertices/faces; process/repair are disabled.
                part = trimesh.Trimesh(np.asarray(mesh.vertices)[ids], inverse.reshape(-1, 3), process=False)
                components.append(dict(vertices=np.asarray(part.vertices), closed_oriented_volume=bool(part.is_volume)))
            self._triangle_assets[name] = dict(triangles=tri, tree=tree, components=components)
        return self._triangle_assets[name]

    def _relative_key(self, a, b, q):
        pair = (a, b)
        if pair not in self._relative_paths:
            incoming = {e["child"]: e for e in self.model.edges}
            chains = []
            for name in pair:
                node, joints = self.model.by_name[name], set()
                while node != self.model.base:
                    edge = incoming[node]
                    if edge["moving"]:
                        joints.add(edge["name"])
                    node = edge["parent"]
                chains.append(joints)
            self._relative_paths[pair] = tuple(sorted(chains[0] ^ chains[1]))
        return pair + tuple(float(q[n]) for n in self._relative_paths[pair])

    def _surface_separated(self, a, b, transforms, q, deadline):
        """Complete original surfaces plus containment exclusion, no mesh edits.

        Every connected closed component must have a vertex outside the other
        mesh's complete AABB. Combined with separated surfaces this excludes
        one solid being nested inside the other; surface distance alone cannot.
        """
        key = self._relative_key(a, b, q)
        if key in self._surface_certificates:
            return self._surface_certificates[key]
        aa, bb = self._triangles(a), self._triangles(b)
        if not all(c["closed_oriented_volume"] for x in (aa, bb) for c in x["components"]):
            return dict(status="unknown", reason="visual_component_not_closed_oriented_volume")
        ta, tb = transforms[a], transforms[b]
        relative_r = ta[:3, :3].T @ tb[:3, :3]
        relative_p = ta[:3, :3].T @ (tb[:3, 3] - ta[:3, 3])
        atri = aa["triangles"]
        btri = bb["triangles"] @ relative_r.T + relative_p
        bbox = np.stack((btri.min((0, 1)), btri.max((0, 1))))
        abox = np.stack((atri.min((0, 1)), atri.max((0, 1))))
        for components, obstacle, move in ((aa["components"], bbox, False), (bb["components"], abox, True)):
            for component in components:
                vertices = component["vertices"]
                if move:
                    vertices = vertices @ relative_r.T + relative_p
                outside = np.any(vertices.min(0) < obstacle[0] - GUARD) or np.any(vertices.max(0) > obstacle[1] + GUARD)
                if not outside:
                    return dict(status="unknown", reason="visual_component_containment_not_excluded")
        candidates = []
        for j, (low, high) in enumerate(zip(btri.min(1), btri.max(1))):
            if time.monotonic() >= deadline:
                return dict(status="unknown", reason="time_bound_in_complete_visual_surface_check")
            candidates.extend((i, j) for i in aa["tree"].intersection(tuple(np.r_[low - GUARD, high + GUARD])))
        for start in range(0, len(candidates), 2048):
            if time.monotonic() >= deadline:
                return dict(status="unknown", reason="time_bound_in_visual_triangle_SAT")
            pairs = np.asarray(candidates[start:start + 2048], dtype=int)
            at, bt = atri[pairs[:, 0]], btri[pairs[:, 1]]
            for triangles in (at, bt):
                if np.any(np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1) < 1e-18):
                    return dict(status="unknown", reason="degenerate_visual_triangle_candidate")
            if np.any(sat_gaps(at, bt) <= GUARD):
                return dict(status="unknown", reason="visual_triangle_separation_not_certified")
        result = dict(status="pass", reason="complete_visual_surfaces_separate_and_component_containment_excluded",
                      triangle_counts=[len(atri), len(btri)], aabb_candidate_pairs=len(candidates),
                      component_counts=[len(aa["components"]), len(bb["components"])])
        self._surface_certificates[key] = result
        return result

    def world(self, q, _unused_fingers=None):
        """The upstream geometry transform, evaluated at the complete USD state."""
        fk = self.model.fk(q)
        world = []
        for shape in self.shapes:
            transform = fk[shape["body_index"]] @ shape["local_transform"]
            r, p = transform[:3, :3], transform[:3, 3]
            if shape["type"] == "Mesh":
                vertices = shape["vertices"] @ r.T + p
                eq = shape["equations"]
                normals = eq[:, :3] @ r.T
                w = dict(type="Mesh", vertices=vertices,
                         equations=np.column_stack((normals, eq[:, 3] - normals @ p)),
                         r=r, center=vertices.mean(0))
            else:
                w = dict(type=shape["type"], center=p, radius=shape["radius"], r=r)
                if shape["type"] == "Cylinder":
                    w.update(axis=r @ shape["axis"], height=shape["height"])
            low, high = self.support(w, np.eye(3))
            w["box"] = np.stack((low, high))
            world.append(w)
        return world

    def check_pose(self, q, *, deadline=None, first_failure=True):
        deadline = time.monotonic() + 10. if deadline is None else float(deadline)
        if set(q) != set(self.revolute_joint_names):
            raise ValueError(f"Expected every USD revolute joint exactly once; missing={sorted(set(self.revolute_joint_names)-set(q))}, extra={sorted(set(q)-set(self.revolute_joint_names))}")
        issues = []
        for name in self.revolute_joint_names:
            value = float(q[name])
            lo, hi = self.model.joints[name]["limits_radians"]
            if not np.isfinite(value) or value < lo - 1e-10 or value > hi + 1e-10:
                issues.append(dict(joint=name, value_rad=value, usd_limits_rad=[lo, hi]))
        if issues:
            return dict(status="limits", issues=issues, complete_pair_check=False)
        result = super().check_pose(q, None, deadline=deadline, first_failure=first_failure)
        result["cooked_status"] = result["status"]
        result["cooked_issues"] = list(result["issues"])
        result["supplemental_visual_status"] = "unchecked"
        result["supplemental_visual_issues"] = []
        result["supplemental_surface_certificates"] = []
        result["supplemental_visual_pairs_checked"] = 0
        result["supplemental_visual_pair_count"] = len(self.visual_pairs)
        result["complete_pair_check"] = False
        if result["status"] != "pass" and first_failure:
            return result
        transforms = self.visual.transforms(q)
        bounds = self.visual.world_bounds(transforms)
        for a, b in self.visual_pairs:
            if time.monotonic() >= deadline:
                result["supplemental_visual_issues"].append(dict(status="unknown", reason="time_bound_in_supplemental_visual_check", bodies=[a, b]))
                break
            result["supplemental_visual_pairs_checked"] += 1
            if self.visual.box_distance(bounds[a], bounds[b]) > GUARD:
                continue
            if self.visual.hull_separated(a, b, transforms, GUARD):
                continue
            surface = self._surface_separated(a, b, transforms, q, deadline)
            if surface["status"] == "pass":
                result["supplemental_surface_certificates"].append(dict(bodies=[a, b], **surface))
                continue
            result["supplemental_visual_issues"].append(dict(status="unknown", reason="visible_mesh_hulls_not_certified_separate", bodies=[a, b],
                                         surface_check=surface, meaning="Neither hull nor complete surface/containment separation certified; mesh collision is not established"))
            if first_failure:
                break
        result["supplemental_visual_status"] = "unknown" if result["supplemental_visual_issues"] else "pass"
        result["issues"].extend(result["supplemental_visual_issues"])
        if not result["issues"]:
            result["complete_pair_check"] = True
        elif result["cooked_status"] != "collision":
            result["status"] = "unknown"
        return result
