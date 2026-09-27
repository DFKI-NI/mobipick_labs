#!/usr/bin/env python3
"""Gazebo ground truth of the tables and objects for RViz (simulation only).

Publishes, for every Gazebo model with a URDF on the parameter server:

``~boxes``  (/sim_ground_truth/boxes) visualization_msgs/MarkerArray: an
            oriented box around the model's collision primitives (blue objects,
            grey tables) and its name ``GT <model>``
``~meshes`` (/sim_ground_truth/meshes) visualization_msgs/MarkerArray: the
            model's URDF visuals: textured meshes, or the primitives a
            configurable model (e.g. a table: top + legs) is made of

so a perception result (boxes, point clouds) can be compared with the truth in
the same view.

The URDF of a model is read from ``worlds.<world>.<model>`` of ``~models_file``,
else ``/<model>_description``, ``/<label>_description`` (label = model name
without ``_<n>``) and, for tables, ``/table_description``. Models whose
collision is a mesh take their box from ``box_overrides``; models without a
URDF are drawn as a small sphere.

Lazy: it subscribes to /gazebo/model_states only while someone subscribes to
one of the topics and republishes at most ``~rate`` Hz, only when a model moved.
On a real robot there are no model states, so it publishes nothing.
"""
import re
import threading

import numpy as np
import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

WORLD_MODELS = {"pbr_cic": "cic_tables", "pbr_moelk": "moelk_tables"}
TABLE_RE = re.compile(r"table", re.IGNORECASE)
IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


def label_of(name):
    return re.sub(r"_\d+$", "", name)


def origin_matrix(origin):
    m = np.eye(4)
    if origin is not None:
        m = euler_matrix(*(origin.rpy or [0.0, 0.0, 0.0]))
        m[:3, 3] = origin.xyz or [0.0, 0.0, 0.0]
    return m


def parse_urdf(xml):
    """Visuals (transform, geometry, rgba) and collision box (lo, hi) of a URDF.

    Transforms are in the root link frame, which is the Gazebo model frame.
    Joints are taken at zero (the sim models only have fixed joints). The box
    is the bounding box of the collision primitives, None if there are none.
    """
    from urdf_parser_py.urdf import URDF, Box, Cylinder, Sphere
    robot = URDF.from_xml_string(xml)
    materials = {m.name: m for m in robot.materials}
    children = {}
    for joint in robot.joints:
        children.setdefault(joint.parent, []).append(joint)
    root = robot.get_root()
    link_tf, stack = {root: np.eye(4)}, [root]
    while stack:
        parent = stack.pop()
        for joint in children.get(parent, []):
            link_tf[joint.child] = link_tf[parent].dot(origin_matrix(joint.origin))
            stack.append(joint.child)

    visuals, corners = [], []
    for link in robot.links:
        if link.name not in link_tf:
            continue
        for visual in link.visuals:
            rgba, mat = None, visual.material
            if mat is not None:
                mat = materials.get(mat.name, mat)
                if mat.color is not None:
                    rgba = list(mat.color.rgba)
            visuals.append((link_tf[link.name].dot(origin_matrix(visual.origin)), visual.geometry, rgba))
        for collision in link.collisions:
            g = collision.geometry
            if isinstance(g, Box):
                half = np.asarray(g.size) / 2.0
            elif isinstance(g, Cylinder):
                half = np.array([g.radius, g.radius, g.length / 2.0])
            elif isinstance(g, Sphere):
                half = np.array([g.radius] * 3)
            else:
                continue
            t = link_tf[link.name].dot(origin_matrix(collision.origin))
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        corners.append(t.dot(np.append(half * [sx, sy, sz], 1.0))[:3])
    box = (np.min(corners, axis=0), np.max(corners, axis=0)) if corners else None
    return visuals, box


class GroundTruthViz:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.rate = float(rospy.get_param("~rate", 1.0))
        self.min_motion = float(rospy.get_param("~min_motion", 0.005))
        config = {}
        models_file = rospy.get_param("~models_file", "")
        if models_file:
            with open(models_file) as f:
                config = yaml.safe_load(f) or {}
        self.worlds = config.get("worlds") or {}
        self.box_overrides = config.get("box_overrides") or {}
        self.exclude = set(config.get("exclude") or [])
        self.box_pub = rospy.Publisher("~boxes", MarkerArray, queue_size=1, latch=True)
        self.mesh_pub = rospy.Publisher("~meshes", MarkerArray, queue_size=1, latch=True)
        self.models = {}  # model name -> (visuals, box) or None
        self.lock = threading.Lock()
        self.latest = None
        self.sub = None
        self.sent = {"boxes": None, "meshes": None}  # model positions last sent per topic
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate, 0.1)), self.tick)

    # ------------------------------------------------------------------ inputs

    def on_states(self, msg):
        with self.lock:
            self.latest = msg

    def tick(self, _event):
        want = {"boxes": self.box_pub.get_num_connections() > 0,
                "meshes": self.mesh_pub.get_num_connections() > 0}
        for topic, wanted in want.items():
            if not wanted:
                self.sent[topic] = None  # a new viewer gets a fresh message
        if not any(want.values()):
            if self.sub is not None:
                self.sub.unregister()
                self.sub = None
            return
        if self.sub is None:
            self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_states,
                                        queue_size=1, buff_size=2 ** 20)
        with self.lock:
            msg = self.latest
        if msg is None:
            return
        poses = {n: (p.position.x, p.position.y, p.position.z) for n, p in zip(msg.name, msg.pose)}
        for topic, pub, build in (("boxes", self.box_pub, self.box_markers),
                                  ("meshes", self.mesh_pub, self.mesh_markers)):
            if want[topic] and not self.unchanged(poses, self.sent[topic]):
                pub.publish(build(msg))
                self.sent[topic] = poses

    def unchanged(self, poses, sent):
        return sent is not None and set(poses) == set(sent) and all(
            max(abs(a - b) for a, b in zip(poses[n], sent[n])) < self.min_motion for n in poses)

    # ------------------------------------------------------------------ models

    def model(self, name, world):
        if name in self.models:
            return self.models[name]
        params = []
        mapped = (self.worlds.get(world) or {}).get(name)
        if mapped:
            params.append(mapped)
        params += ["%s_description" % name, "%s_description" % label_of(name)]
        if TABLE_RE.search(name):
            params.append("table_description")
        parsed = None
        for param in params:
            param = param if param.startswith("/") else "/" + param
            if rospy.has_param(param):
                try:
                    parsed = parse_urdf(rospy.get_param(param))
                except Exception as exc:  # noqa: BLE001 - a broken URDF only loses its drawing
                    rospy.logwarn("sim_ground_truth_viz: cannot parse %s: %s", param, exc)
                break
        override = self.box_overrides.get(name) or self.box_overrides.get(label_of(name))
        if override:
            offset = np.asarray(override.get("center_offset", [0.0, 0.0, 0.0]), dtype=float)
            half = np.asarray(override["size"], dtype=float) / 2.0
            parsed = (parsed[0] if parsed else [], (offset - half, offset + half))
        self.models[name] = parsed
        return parsed

    def world_of(self, msg):
        return next((WORLD_MODELS[n] for n in msg.name if n in WORLD_MODELS), "")

    def drawn(self, msg):
        """(index, name, model transform, parsed model) of every model to draw."""
        world = self.world_of(msg)
        for i, (name, pose) in enumerate(zip(msg.name, msg.pose)):
            if name in self.exclude:
                continue
            q, p = pose.orientation, pose.position
            tf = quaternion_matrix([q.x, q.y, q.z, q.w])
            tf[:3, 3] = [p.x, p.y, p.z]
            yield i, name, tf, self.model(name, world)

    # ------------------------------------------------------------------ markers

    def marker(self, ns, mid, mtype):
        m = Marker(ns=ns, id=mid, type=mtype, action=Marker.ADD)
        m.header.frame_id = self.frame_id
        m.pose.orientation.w = 1.0
        return m

    def cleared(self):
        out = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = self.frame_id
        out.markers.append(clear)
        return out

    def box_markers(self, msg):
        out = self.cleared()
        for i, name, tf, parsed in self.drawn(msg):
            table = bool(TABLE_RE.search(name))
            box = parsed[1] if parsed else None
            if box is not None:
                lo, hi = box
                center = tf.dot(np.append((lo + hi) / 2.0, 1.0))[:3]
                m = self.marker("tables" if table else "objects", i, Marker.CUBE)
                m.pose = Pose(Point(*center), Quaternion(*quaternion_from_matrix(tf)))
                m.scale = Vector3(*np.maximum(hi - lo, 0.005))
                m.color = ColorRGBA(0.75, 0.75, 0.8, 0.25) if table else ColorRGBA(0.3, 0.3, 1.0, 0.45)
                out.markers.append(m)
                # top of the rotated box
                top = center[2] + 0.5 * float(np.abs(tf[2, :3]).dot(hi - lo))
            else:
                m = self.marker("objects", i, Marker.SPHERE)
                m.pose = Pose(Point(*tf[:3, 3]), IDENTITY)
                m.scale = Vector3(0.05, 0.05, 0.05)
                m.color = ColorRGBA(0.3, 0.3, 1.0, 0.8)
                out.markers.append(m)
                center, top = tf[:3, 3], tf[2, 3] + 0.03
            text = self.marker("labels", i, Marker.TEXT_VIEW_FACING)
            text.pose = Pose(Point(center[0], center[1], top + 0.04), IDENTITY)
            text.scale.z = 0.035
            text.color = ColorRGBA(0.75, 0.75, 1.0, 1.0)
            text.text = "GT " + name + ("" if box is not None else " (no size)")
            out.markers.append(text)
        return out

    def mesh_markers(self, msg):
        from urdf_parser_py.urdf import Box, Cylinder, Mesh, Sphere
        out = self.cleared()
        mid = 0
        for _i, _name, model_tf, parsed in self.drawn(msg):
            for local_tf, geom, rgba in (parsed[0] if parsed else []):
                if isinstance(geom, Mesh):
                    m = self.marker("meshes", mid, Marker.MESH_RESOURCE)
                    m.mesh_resource = geom.filename
                    m.mesh_use_embedded_materials = True
                    m.scale = Vector3(*(geom.scale or [1.0, 1.0, 1.0]))
                    m.color = ColorRGBA(0, 0, 0, 0)  # all zero: keep the mesh's own materials
                else:
                    if isinstance(geom, Box):
                        m = self.marker("meshes", mid, Marker.CUBE)
                        m.scale = Vector3(*geom.size)
                    elif isinstance(geom, Cylinder):
                        m = self.marker("meshes", mid, Marker.CYLINDER)
                        m.scale = Vector3(2 * geom.radius, 2 * geom.radius, geom.length)
                    elif isinstance(geom, Sphere):
                        m = self.marker("meshes", mid, Marker.SPHERE)
                        m.scale = Vector3(*([2 * geom.radius] * 3))
                    else:
                        continue
                    c = rgba or [0.55, 0.53, 0.5, 1.0]
                    m.color = ColorRGBA(c[0], c[1], c[2], 0.85)
                t = model_tf.dot(local_tf)
                m.pose = Pose(Point(*t[:3, 3]), Quaternion(*quaternion_from_matrix(t)))
                out.markers.append(m)
                mid += 1
        return out


if __name__ == "__main__":
    rospy.init_node("sim_ground_truth_viz")
    GroundTruthViz()
    rospy.spin()
