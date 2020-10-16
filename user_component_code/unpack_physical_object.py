﻿# MIT License
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

"""Unpacks a physical object into its geometries and transformation.
    Inputs:
        physical_object: Physical object to unpack.
    Output:
        convex_geometries:  Convex geometries of the physical object.
        transformation:     Transformation of the physical object."""

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
    if physical_object is None:
        ghsim.log_error(ghenv, 
            "The \"physical_object\" parameter must not be empty.")
    else:
        convex_geometries = physical_object.physical_object_type.convex_geometries
        transformation = physical_object.transformation
