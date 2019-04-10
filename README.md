# uw_dynamics
## Abstract:
Underwater biomimetic robots are biologically inspired robots that involve complex rigid body dynamics and fluid-structure interactions. Simulation of such robots with gazebo has always been a challenging task. Unlike underwater vehicles such as ROVs and AUVs, these robots are propelled by fluid-structure interaction which are relatively hard to simulate compared to thrusters/propellers (as found in ROVs and AUVs). The proposed project aims at developing a gazebo package containing plugins capable of simulating such robots. It can also be extended to simulate biomimetic robots in fluids of various densities and even aerial biomimetic robots.

## Getting Started:
Clone the repository to the home directory,
```
$ git clone https://github.com/imsenthur/uw_dynamics.git
```
## Build:
```
$ cd uw_dynamics/
$ mkdir build
$ cd build/
$ cmake ../
$ make
```
## Add it to your GAZEBO_PLUGIN_PATH:
```
$ export GAZEBO_PLUGIN_PATH=~/uw_dyn/build/devel/lib:$GAZEBO_PLUGIN_PATH
```
## Implementation:
```
<gazebo>
   <plugin name="underwater_dynamics" filename="libuw_dynamics.so">
     <fluid_density>997</fluid_density>
     <link_name>base_link</link_name>
     <com>0 0 0</com>
     <!-- will be automatically computed if not specified -->
     <cdx1>0.1</cdx1>
     <cdy1>0.3</cdy1>
     <cdx2>0.2</cdx2>
     <cdy2>0.1</cdy2>
   </plugin>
</gazebo>
```
*Add the plugin to your robot's URDF/SDF.*
![alt text](https://raw.githubusercontent.com/imsenthur/uw_dynamics/master/sim.png)
