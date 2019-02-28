# Opal

Opal is a library to simulate multipath channel propagation with NVIDIA Optix for multiple moving transmitters and receivers.
Opal is a 3D ray-launching based, deterministic radio-
frequency propagation simulator. With Ray-launching methods, 
also called shooting and bouncing (SBR), electromagnetic waves are simulated by rays launched from
the transmitter with a predetermined angular spacing (AS).
These rays are propagated along their trajectory until they hit
an obstacle where they are reflected, diffracted, transmitted
or scattered. Subsequent rays are traced again. The contri-
butions of the different rays that hit a reception sphere on
the receiver are added to compute the electric field. 

It uses the [NVIDIA Optix ray-tracing engine](https://developer.nvidia.com/optix) to propagate the rays in a 
dynamical scene, 
which is built from meshes loaded from files or from another application, such 
as a game engine.



It can be used as a standalone application or as a Unity plugin. If used as
a Unity plugin, a Unity program will send the meshes and update the transforms 
of the elements of the scene. The Unity related code can be found in 
our   [Veneris repository](https://gitlab.com/esteban.egea/veneris).

## Installation

### Requirements
You need a modern NVIDIA GPU and updated driver.  CUDA 9.0 or later. Optix 5.1. 
CMake 3.10

### Install Optix
Download Optix from the [NVIDIA site]((https://developer.nvidia.com/optix) and 
follow instructions to install it. It has been developed with Optix 5.1.
Basically you need to install CUDA first, which for Optix 5.1 should be 9.0 
although it has been tested with 9.1 and 9.2. Do not use CUDA 10.0, not tested 
with this version of Opal.

Unzip Optix. 



```bash
pip install foobar
```

## Usage

```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)