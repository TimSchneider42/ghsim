#! /usr/bin/env python

# MIT License
#
# Copyright (c) 2020 Tim Schneider
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import argparse
import os
import pickle
import shutil
import socket
import struct
import time
from tempfile import TemporaryDirectory
from typing import List, Optional, Any, Tuple, Dict

import pybullet as p
import numpy as np
import pybullet_data
from scipy.spatial.transform import Rotation

VERSION = (1, 1, 0, 0)


class Connection:
    """
    Represents a socket connection to a client.
    """
    def __init__(self, s: socket):
        self._socket = s
        self._buffer = bytearray()

    def receive_object(self) -> Optional[Any]:
        """
        Receives the next pickled object from the input buffer.
        :return: The unpickled object.
        """
        while len(self._buffer) < 4 or len(self._buffer) < struct.unpack("<L", self._buffer[:4])[0] + 4:
            new_bytes = self._socket.recv(16)
            if len(new_bytes) == 0:
                return None
            self._buffer += new_bytes

        length = struct.unpack("<L", self._buffer[:4])[0]
        header, body = self._buffer[:4], self._buffer[4:length + 4]

        obj = pickle.loads(body)

        self._buffer = self._buffer[length + 4:]

        return obj

    def send_object(self, d: Any):
        """
        Sends an object to the client.
        :param d: Object to send. Must be picklable.
        :return:
        """
        body = pickle.dumps(d, protocol=2)
        header = struct.pack("<L", len(body))
        msg = header + body
        self._socket.send(msg)


def to_rhino_transforms(pybullet_poses: List[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]) \
        -> List[List[List[float]]]:
    """
    Converts the given PyBullet poses to transformation matrices that Rhino can process.
    :param pybullet_poses: PyBullet poses (position + quaternion) to convert.
    :return: Transformation matrices.
    """
    rotation_matrices = np.array([Rotation.from_quat(q).as_matrix() for t, q in pybullet_poses])
    translations = np.array([t for t, q in pybullet_poses])

    transforms = np.tile(np.eye(4)[np.newaxis], reps=(len(pybullet_poses), 1, 1))
    transforms[:, :3, 3] = translations * 1000
    transforms[:, :3, :3] = rotation_matrices
    return transforms.tolist()


def simulate(physical_objects: List[Dict], sim_length_s: float = 10.0, time_step_s: float = 1 / 240, substeps: int = 1,
             real_time_factor: Optional[float] = None, ground_plane_pos: Optional[float] = None) \
        -> List[List[List[List[float]]]]:
    """
    Simulates the given configuration.
    :param physical_objects:    Physical objects to simulate.
    :param sim_length_s:        Length of the simulation in seconds.
    :param time_step_s:         Time step of the simulator.
    :param substeps:            Subdivide time step of the simulator by this factor.
    :param real_time_factor:    Factor of real time to run simulation in. Leave "None" for simulation at maximum speed.
    :param ground_plane_pos:    Position of the ground plane on the Z-Axis. If left None, it is automatically placed
                                3mm below the lowest point of any object.
    :return: List of list of poses of each object at each time step.
    """
    print("Simulating...")
    p.setPhysicsEngineParameter(numSubSteps=substeps)
    min_z_pos = np.infty
    bodies = [None] * len(physical_objects)
    for i, po in enumerate(physical_objects):
        trans = po["transformation"]
        h = po["mesh_hash"]
        mass = po["mass_kg"] if "mass_kg" in po else 1
        trans = np.array(trans) * 0.001
        pos = trans[:3, 3]
        rotation = Rotation.from_matrix(trans[:3, :3])
        bodies[i] = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=mesh_store[h], basePosition=pos,
                                      baseOrientation=rotation.as_quat())
        dynamics_kwargs = po.get("dynamics", {})

        if len(dynamics_kwargs) > 0:
            p.changeDynamics(bodies[i], linkIndex=-1, **dynamics_kwargs)
        min_pos, max_pos = p.getAABB(bodies[i])
        min_z_pos = np.minimum(min_pos[2], min_z_pos)

    if ground_plane_pos is None:
        ground_plane_pos_scaled = min_z_pos - 0.003
    else:
        ground_plane_pos_scaled = ground_plane_pos / 1000
    p.resetBasePositionAndOrientation(plane, [0, 0, ground_plane_pos_scaled], [0.0, 0.0, 0.0, 1.0])
    p.setTimeStep(time_step_s)

    results = [to_rhino_transforms([p.getBasePositionAndOrientation(b) for b in bodies])]

    next_step = time.time()
    for i in range(int(sim_length_s / time_step_s)):
        if real_time_factor is not None:
            time.sleep(max(next_step - time.time(), 0))
            next_step += time_step_s / real_time_factor
        p.stepSimulation()
        results.append(to_rhino_transforms([p.getBasePositionAndOrientation(b) for b in bodies]))

    # Cleanup
    for b in bodies:
        p.removeBody(b)

    print("Done")

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Aggregation Simulation Server")
    parser.add_argument("-e", "--expose", action="store_true", help="Expose this server to the network "
                                                                    "(DO NOT ENABLE IN PUBLIC NETWORKS).")
    parser.add_argument("-p", "--port", type=int, default=8000, help="Port this server listens to.")
    parser.add_argument("-v", "--visualize", action="store_true", help="Visualize the simulation with pybullet's built-"
                                                                       "in visualizer.")
    parser.add_argument("-r", "--real-time-factor", type=float,
                        help="Real time speed factor of the simulation. Setting 0.5 here means that the simulation runs"
                             " in 50% of real time. Can be overwritten from Grasshopper. This option is ignored if -v"
                             " is not set. Default: no real time factor (simulate as fast as possible).")
    args = parser.parse_args()

    version_st = tuple(map(str, VERSION))
    print("Simulation Server v. {}".format(".".join(version_st)))

    active_connections = {}

    mesh_store = {}

    physicsClient = p.connect(p.GUI if args.visualize else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])

    world_state = p.saveState()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("0.0.0.0" if args.expose else "localhost", args.port))
        s.listen(5)
        s.settimeout(0)

        print("Listening on port {}...".format(args.port))

        with TemporaryDirectory() as tmpdir:
            while True:
                try:
                    clientsocket, address = s.accept()
                    print("Client connected from {}:{}".format(*address))
                    clientsocket.settimeout(0)
                    c = Connection(clientsocket)
                    active_connections[clientsocket] = (c, time.time())
                except BlockingIOError:
                    pass

                delete_list = []
                now = time.time()
                for clientsocket, (conn, t) in active_connections.items():
                    try:
                        m = conn.receive_object()
                    except (BlockingIOError, ConnectionResetError):
                        m = None
                    if m is not None:
                        if "version" not in m or m["version"][:-2] != VERSION[:-2]:
                            error = "Received request with mismatched version (v. {} while my version is {}). " \
                                    "Please make sure that you are running matching versions of the simulation server " \
                                    "and the Grasshopper client (either both {}.x.x or both {}.x.x).".format(
                                ".".join(map(str, m["version"])), ".".join(version_st), ".".join(version_st[:-2]),
                                ".".join(map(str, m["version"][:-2])))
                            print(error)
                            conn.send_object(
                                {"type": "error", "err": "VERSION_MISMATCH", "desc": error, "version": VERSION})
                        elif m["type"] == "sim":
                            # Check if all meshes are present
                            mesh_hashes = set(part["mesh_hash"] for part in m["objects"])
                            unknown_hashes = [m for m in mesh_hashes if not m in mesh_store]
                            if len(unknown_hashes) > 0:
                                conn.send_object({"type": "mesh_request", "hashes": mesh_hashes})
                            else:
                                # Simulate
                                d = {
                                    "sim_length_s": 5.0,
                                    "time_step_s": 0.1,
                                    "substeps": 20
                                }
                                d.update(m)
                                if args.visualize:
                                    rtf = args.real_time_factor
                                    if "real_time_factor" in d:
                                        rtf = d["real_time_factor"]
                                else:
                                    rtf = None
                                result = simulate(d["objects"], d["sim_length_s"], d["time_step_s"], d["substeps"], rtf, 
                                    d["ground_plane_pos"])
                                conn.send_object({"type": "sim_result", "poses": result})
                        elif m["type"] == "meshes":
                            for h, mesh_lst in m["meshes"].items():
                                if not h in mesh_store:
                                    d = os.path.join(tmpdir, h)
                                    if os.path.exists(d):
                                        shutil.rmtree(d)
                                    os.mkdir(d)
                                    for i, mesh in enumerate(mesh_lst):
                                        with open(os.path.join(d, "{}.obj".format(i)), "w") as f:
                                            f.write("o {}_{}\n".format(h, i))
                                            for v in mesh["vertices"]:
                                                f.write("v {:0.8f} {:0.8f} {:0.8f}\n".format(*(np.array(v) * 0.001)))
                                            for fa in mesh["faces"]:
                                                f.write("f {} {} {}\n".format(*(np.array(fa) + 1)))
                                    mesh_store[h] = p.createCollisionShapeArray(
                                        [p.GEOM_MESH] * len(mesh_lst),
                                        fileNames=[os.path.join(d, "{}.obj".format(i)) for i in
                                                   range(len(mesh_lst))])

                                # In case that ever gets fixed
                                # mesh_store[h] = p.createCollisionShapeArray(
                                #     [p.GEOM_MESH] * len(mesh_lst),
                                #     vertices=[[[ei * 0.001 for ei in e] for e in m["vertices"]] for m in mesh_lst],
                                #     indices=[[ei for e in m["faces"] for ei in e] for m in mesh_lst])
                        else:
                            print("Unknown message type \"{}\"".format(m["type"]))

                    if now - t > 60:
                        delete_list.append(clientsocket)

                for cs in delete_list:
                    cs.close()
                    del active_connections[cs]

                time.sleep(0.1)
    finally:
        try:
            s.shutdown(socket.SHUT_RDWR)
        finally:
            s.close()
