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

"""Creates a type of physical object for simulation.
    Inputs:
        name:               Name of this physical object type.
        convex_geometries:  Convex geometric shapes this physical object type 
                            consists of. Note that using non-convex geometric
                            shapes is generally possible but might result in an
                            unstable stable simulation. Instead, decompose any
                            geometric shapeinto multiple convex shapes before 
                            using it in the simulation.
        mass_kg:            Mass of this physical object type in kg (default 
                            1kg).
        lateral friction:   Friction of this object sliding over another object.
        spinning friction:  Friction of this object spinning on another object
                            (e.g. a top spinning on a table).
        rolling friction:   Friction of an object rolling over another object 
                            (e.g. a wheel rolling over the ground).
                            For details on how to set these values, please refer
                            to the documentation of the bullet physics engine.
    Output:
        physical_object_type: A physical object type."""
        
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
    ok = True
    if name is None:
        ok = False
        ghsim.log_error(ghenv, "The \"name\" parameter must not be empty.")
    if convex_geometries is None:
        ok = False
        ghsim.log_error(ghenv, 
            "The \"convex_geometries\" parameter must not be empty.")
    if mass_kg is None:
        mass_kg = 1.0
    dynamics = {}
    if lateral_friction is not None:
        dynamics["lateralFriction"] = lateral_friction
    if spinning_friction is not None:
        dynamics["spinningFriction"] = spinning_friction
    if rolling_friction is not None:
        dynamics["rollingFriction"] = rolling_friction

if ok:
    physical_object_type = ghsim.PhysicalObjectType(name, convex_geometries,
        mass_kg, dynamics)
        
