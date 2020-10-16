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

"""Simulates the behavior of the given physical objects.
    Inputs:
        physical_objects:       The physical objects to simulate.
        simulator:              The simulator configuration.
        enabled:                Set this to false to disable the simulation.
        simulation_time_s:      Total time of the simulation in seconds (default
                                10s).
        simulation_time_step_s: Time step of the simulator in seconds (default
                                0.1s).
        real_time_factor:       Real time factor of the simulation. Setting e.g.
                                0.5 here will cause the simulation to run in
                                half of real time speed. Note that this
                                parameter is ignored if the simulation server
                                was not launched with the "-v" flag.
        simulation_substeps:    Further subdivide the simulator steps by this
                                number (default 20). This will trade performance
                                over accuracy.
        ground_plane_pos:       Position of the ground plane (z coordinates)
                                (default: lowest point of all meshes -3mm).
    Output:
        simulation_result:  The results of the simulation as a list of 
                            simulation states (one for each time step)."""

__author__ = "Tim Schneider"
__version__ = "2020.10.12"

import rhinoscriptsyntax as rs
import Grasshopper as gh

import sys

ghcompfolder = gh.Folders.DefaultAssemblyFolder
ghsim_path = ghcompfolder + "ghsim"
if ghsim_path not in sys.path:
    sys.path.append(ghsim_path)
try:
    import ghsim
except:
    ghenv.Component.AddRuntimeMessage(gh.Kernel.GH_RuntimeMessageLevel.Error, 
        "Cannot import ghsim. Please ensure that the ghsim.py "
        "module is installed in \"{}\".".format(ghsim_path))
    ghsim = None

if ghsim:
    if enabled:
        ok = True
        if simulation_time_s is None:
            simulation_time_s = 5.0
        if simulation_time_step_s is None:
            simulation_time_step_s = 0.1
        if simulation_substeps is None:
            simulation_substeps = 20
        if physical_objects is None:
            ok = False
            ghsim.log_error(ghenv, 
                "The \"physical_objects\" parameter must not be empty.")
        if simulator is None:
            ok = False
            ghsim.log_error(ghenv, "The \"simulator\" parameter must not be empty.")
        if ok:
            try:
                with ghsim.SimulatorConnection(simulator) as conn:
                    simulation_result = conn.simulate(physical_objects, 
                        simulation_time_s, simulation_time_step_s, 
                        simulation_substeps, real_time_factor, ground_plane_pos)
            except Exception as e:
                ghsim.log_error(ghenv, str(e))