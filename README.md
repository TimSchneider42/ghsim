# ghsim
Bullet simulation for Rhino Grasshopper.

## Installation
1. Install `python 3.6` or higher on your system.
The newest version of python can be obtained from <https://www.python.org/downloads/>.

2. Install dependencies for the simulation server.<br>
```pip install numpy scipy```

3. Install the `PyBullet` simulator.
This can be achieved by opening the terminal and typing<br>
```pip install pybullet```<br>
However, this will install the upstream version of `PyBullet` which currently limits the number of meshes of a single physical object to 16.
If this is not sufficient for you, you can install our fork instead, which increases the limit to 32:
    1. Download the sources from <https://github.com/TimSchneider42/bullet3/archive/d3c152d77f30ffc837be36710563d6dede01fb56.zip>
    2. Unpack the sources.
    3. Run `pip install /path/to/unpacked/sources`
    
4. Install the `ghsim` Grasshopper plugin.
    1. Open Grasshopper and click on File > Special Folders > Components Folder
    2. Copy the `ghsim` folder (the directory containing `ghsim.py`, not the entire repository) into the directory that just opened.
    In the components folder of Grasshopper should afterwards be a directory `ghsim` containing only a single file `ghsim.py`.
    3. Again in Grasshopper, click File > Special Folders > User Objects
    4. Copy the content (!) of the `user_components` directory into the directory that just opened.
    
    You should now see a category `ghsim` as soon as you restart Rhino/Grasshopper.
    
## Usage
Before the simulation can begin, the simulation server needs to be started.
This can be achieved by opening a terminal, navigating to the source root of this repository and typing

```python simulation_server.py```

For a list of options, type 

```python simulation_server.py -h```

As soon as the server outputs `Listening on port 8000...`, it is ready to process simulation requests.

`ghsim` comes with 6 types of Grasshopper components, that are briefly explained here:

- **PhysicalObjectType**: Creates a type of physical object that can appear multiple times at different locations in the scene. 
Each type consists of a one or more convex geometric shapes that remain in static formation during simulation, and dynamic properties, like the mass or friction coefficients.
- **PhysicalObject**: Creates an instance of a physical object type. 
Each physical object consists of the respective object type and a transformation specifying its location.
- **ExternalSimulator**: Specifies hostname and port of the simulation server.
- **Simulation**: Executes the simulation given a list of physical objects and some simulation parameters.
For further information on the simulation parameters, please refer to the documentation of this component.
Outputs a simulation result object for each time step of the simulation.
- **SimulationStateAt**: Extracts the transformation of each physical object at a specific time from the simulator output.
- **UnpackPhysicalObject**: Unpacks a physical object into its convex geometries and its transformation.

For an usage example, please refer to `boxes.gh` and `wasp.gh`.

### Simulation Advice

Ensure that any objects in the scene are not too close together at the beginning of the simulation.
Otherwise they might start to overlap which makes the simulation unstable.
Instead, always leave 1-2mm distance between objects and rather let them settle during the simulation. 
