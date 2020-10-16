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

import pickle
import hashlib

import socket
import struct

import Rhino
import Grasshopper as gh


class SimulatorConnection:
    """
    Models a socket-based connection to the simulation server.
    """
    VERSION = (1, 1, 0, 0)

    def __init__(self, simulator):
        """

        :param simulator: Simulator configuration to use.
        """
        self._socket = None
        self._simulator = simulator
        self._buffer = ""

    def _receive_object(self):
        """
        Receives the next pickled object from the input buffer.
        :return:
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

    def _send_object(self, d):
        """
        Sends the given object.
        :param d: Object to send. Must be picklable.
        :return:
        """
        body = pickle.dumps(d)
        header = struct.pack("<L", len(body))
        msg = header + body
        self._socket.send(msg)

    def __enter__(self):
        """
        Establishes a connection with the configured server.
        :return:
        """
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self._simulator.hostname, int(self._simulator.port)))
            return self
        except:
            raise ValueError("Connection to simulation server could not be established.")

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Disconnects the connection.
        :param exc_type:
        :param exc_value:
        :param exc_traceback:
        :return:
        """
        self._socket.shutdown(socket.SHUT_RDWR)
        self._socket.close()

    def _send_meshes(self, physical_objects):
        """
        Sends the meshes of the given physical objects to the server where they are cached.
        :param physical_objects: Physical objects to send meshes of.
        :return:
        """
        physical_object_types = set(p.physical_object_type for p in physical_objects)
        output_dict = {
            "type": "meshes",
            "meshes": {
                p.mesh_hash: p.mesh_dicts for p in physical_object_types
            },
            "version": self.VERSION
        }
        self._send_object(output_dict)

    def _to_rhino_transform(self, transform):
        """
        Converts a regular transformation matrix to a Rhino transformation.
        :param transform: Transformation matrix to convert.
        :return: The Rhino transformation.
        """
        rt = Rhino.Geometry.Transform()
        for i in range(4):
            for j in range(4):
                rt[i, j] = transform[i][j]
        return rt

    def _from_rhino_transform(self, rhino_transform):
        """
        Converts a Rhino transformation to a transformation matrix.
        :param rhino_transform: Rhino transformation to convert.
        :return: Transformation matrix.
        """
        return [[float(rhino_transform[i, j]) for j in range(4)] for i in range(4)]

    def _send_simulation(self, physical_objects, simulation_time_s, simulation_time_step_s, simulation_substeps,
                         real_time_factor, ground_plane_pos):
        """
        Sends a simulation request.
        :param physical_objects:        Physical objects to simulate.
        :param simulation_time_s:       Total length of the simulation.
        :param simulation_time_step_s:  Simulator time step to use.
        :param simulation_substeps:     Subdivide the simulation time step by this factor. Note that substeps will not
                                        be included in the output of the simulator.
        :param real_time_factor:        Factor of real time to run simulation in. Leave "None" for simulation at
                                        maximum speed.
        :param ground_plane_pos:        Position of the ground plane on the Z-axis.
        :return:
        """
        object_props = [
            dict({
                "transformation": self._from_rhino_transform(po.transformation),
                "mesh_hash": po.physical_object_type.mesh_hash,
                "mass_kg": po.physical_object_type.mass_kg,
                "dynamics": po.physical_object_type.dynamic_properties
            })
            for po in physical_objects
        ]

        output_dict = {
            "type": "sim",
            "sim_length_s": simulation_time_s,
            "time_step_s": simulation_time_step_s,
            "substeps": simulation_substeps,
            "objects": object_props,
            "ground_plane_pos": ground_plane_pos,
            "version": self.VERSION
        }
        if real_time_factor is not None:
            output_dict["real_time_factor"] = real_time_factor
        self._send_object(output_dict)

    def simulate(self, physical_objects, simulation_time_s, simulation_time_step_s, simulation_substeps,
                 real_time_factor, ground_plane_pos):
        """
        Executes the simulation.
        :param physical_objects:        Physical objects to simulate.
        :param simulation_time_s:       Total length of the simulation.
        :param simulation_time_step_s:  Simulator time step to use.
        :param simulation_substeps:     Subdivide the simulation time step by this factor. Note that substeps will not
                                        be included in the output of the simulator.
        :param real_time_factor:        Factor of real time to run simulation in. Leave "None" for simulation at
                                        maximum speed.
        :param ground_plane_pos:        Position of the ground plane on the Z-axis.
        :return: List of simulation states for each time step.
        """
        try:
            self._send_simulation(physical_objects, simulation_time_s, simulation_time_step_s, simulation_substeps,
                                  real_time_factor, ground_plane_pos)

            result = self._receive_object()

            if result["type"] == "mesh_request":
                self._send_meshes(physical_objects)
                self._send_simulation(physical_objects, simulation_time_s, simulation_time_step_s, simulation_substeps,
                                      real_time_factor, ground_plane_pos)
                result = self._receive_object()
        except:
            raise ValueError("Communication to simulation server failed.")

        if result["type"] == "error":
            raise ValueError("Received error from server: {}".format(result["desc"]))
        if result["type"] != "sim_result":
            raise ValueError("Received unexpected reply (result type \"{}\")".format(result["type"]))
        return [
            SimulationState(
                i * simulation_time_step_s,
                [PhysicalObject(po.physical_object_type, self._to_rhino_transform(t)) for t, po in
                 zip(s, physical_objects)])
            for i, s in enumerate(result["poses"])]


class PhysicalObjectType(object):
    """
    Represents a type of physical object.
    """
    def __init__(self, name, convex_geometries, mass_kg, dynamic_properties):
        """

        :param name:                Name of this physical object type.
        :param convex_geometries:   Convex geometries this physical object type consists of. Providing non-convex
                                    geometries here might cause an unstable simulation.
        :param mass_kg:             Mass of this physical object type in kg.
        :param dynamic_properties:  Dynamic properties of this physical object type. The values passed in this
                                    dictionary are directly passed to PyBullet.changeDynamics. For further details on
                                    which parameters can be configured here, please refer to the PyBullet quickstart
                                    guide.
        """
        self._name = name
        self._convex_geometries = convex_geometries
        self._mass_kg = mass_kg
        self._dynamic_properties = dynamic_properties
        self._mesh_dicts = [self._mesh_to_dict(m) for m in self._convex_geometries]
        self._mesh_hash = hashlib.sha256(pickle.dumps(self._mesh_dicts)).hexdigest()[:16]

    @staticmethod
    def _mesh_to_dict(mesh):
        """
        Converts the given mesh to a dictionary.
        :param mesh: Mesh to convert.
        :return: Dictionary containing the vertices and faces of the mesh.
        """
        mesh.Faces.ConvertQuadsToTriangles()
        vertices = [[float(p.X), float(p.Y), float(p.Z)] for p in mesh.Vertices]
        faces = [[int(f.A), int(f.B), int(f.C)] for f in mesh.Faces]
        return {"vertices": vertices, "faces": faces}

    @property
    def convex_geometries(self):
        return self._convex_geometries

    @property
    def mass_kg(self):
        return self._mass_kg

    @property
    def dynamic_properties(self):
        return self._dynamic_properties

    @property
    def name(self):
        return self._name

    @property
    def mesh_dicts(self):
        """
        Dictionary representations of the meshes.
        :return:
        """
        return self._mesh_dicts

    @property
    def mesh_hash(self):
        """
        Hash of the meshes. Is used for server-side cashing.
        :return:
        """
        return self._mesh_hash

    def __repr__(self):
        return "PhysicalObjectType {}".format(self._name)

    ToString = __repr__


class PhysicalObject(object):
    """
    Represents a physical object.
    """
    def __init__(self, physical_object_type, transformation):
        """

        :param physical_object_type:    Type of this physical object.
        :param transformation:          Transformation of this physical object in the scene.
        """
        self._physical_object_type = physical_object_type
        self._transformation = transformation

    @property
    def physical_object_type(self):
        return self._physical_object_type

    @property
    def transformation(self):
        return self._transformation

    def __repr__(self):
        return "PhysicalObject (type={})".format(self._physical_object_type)

    ToString = __repr__


class Simulator(object):
    """
    Represents the configuration of an external simulator.
    """
    def __init__(self, hostname, port):
        self._hostname = hostname
        self._port = port

    @property
    def hostname(self):
        return self._hostname

    @property
    def port(self):
        return self._port

    def __repr__(self):
        return "Simulator on {}:{}".format(self._hostname, self._port)

    ToString = __repr__


class SimulationState(object):
    """
    Represents the state of the simulation at a specific time step.
    """
    def __init__(self, time_s, physical_objects):
        """
        :param time_s:              Time step of this simulation state.
        :param physical_objects:    List of physical objects in the respective configuration.
        """
        self._time_s = time_s
        self._physical_objects = physical_objects

    @property
    def time_s(self):
        return self._time_s

    @property
    def physical_objects(self):
        return self._physical_objects

    def __repr__(self):
        return "SimulationState (time={}s, {} physical objects)".format(self._time_s, len(self._physical_objects))

    ToString = __repr__


def log_error(ghenv, msg):
    """
    Logs an error such that the user can see it.
    :param ghenv:   ghenv element of the script calling this function.
    :param msg:     Message to log.
    :return:
    """
    ghenv.Component.AddRuntimeMessage(gh.Kernel.GH_RuntimeMessageLevel.Error, msg)
