# Opal

Opal is a library to simulate on GPUs multipath channel propagation with NVIDIA Optix for multiple moving transmitters and receivers.
Opal is a 3D ray-launching based, deterministic radio-
frequency propagation simulator. With Ray-launching methods, 
also called shooting and bouncing (SBR), electromagnetic waves are simulated by rays launched from
the transmitter with a predetermined angular spacing (AS).
These rays are propagated along their trajectory until they hit
an obstacle where they are reflected, diffracted, transmitted
or scattered. Subsequent rays are traced again. The contributions of the different rays that hit a reception sphere on
the receiver are added to compute the electric field. 

It uses the [NVIDIA Optix ray-tracing engine](https://developer.nvidia.com/optix) to propagate the rays in a 
dynamical scene, 
which is built from meshes loaded from files or from another application, such 
as a game engine.

Features:
* Reflections
* Penetration
* Depolarization 
* It supports simultaneous transmissions from multiple transmitters to multiple receivers.
* In future releases we will add difraction.

It can be used as a standalone application or as a Unity plugin. If used as
a Unity plugin, a Unity program will send the meshes and update the transforms 
of the elements of the scene. The Unity related code can be found in 
our   [Veneris repository](https://gitlab.com/esteban.egea/veneris).

For more information visit the [Veneris project website](http://pcacribia.upct.es/veneris).

## Installation

### Requirements
You need a modern NVIDIA GPU and updated driver.  CUDA 9.0 or later. Optix 6.0 or 5.1. gcc or Visual Studio (use a recent version, since it uses nested namespaces. I use g++ 7.4.0).
CMake 3.10

### Install Optix
Download Optix from the [NVIDIA site](https://developer.nvidia.com/optix) and 
follow instructions to install it.

**Do not use Optix 7**. It is not backward compatible and uses a completely different API.

**Updated to Optix 6.0**
It has been tested with the last Optix version, 6.5, and **the performance on the same hardware has improved remarkably**, even without using RTX cores. 
With Optix 6.0 
 use CUDA 10.0 and requires a NVIDIA driver at least 418.30. Follow exactly the same steps as below, but with Optix 6.0 and CUDA 10.0.
 With Optix 6.5 
 use CUDA 10.1 and requires a NVIDIA driver at least 435.17. Follow exactly the same steps as below, but with Optix 6.0 and CUDA 10.0.

With Optix 5.1.
Basically you need to install CUDA first, which for Optix 5.1 should be 9.0 
although it has been tested with 9.1 and 9.2. Do not use CUDA 10.0, does not work with Optix 5.1.

Unzip Optix. Go to the root folder of Optix and build it to test it works. For this, use Cmake as follows:

In Linux: 
```bash
cd SDK
ccmake .
```
Alternatively, as recommended in the INSTALL, you can create your build directory, go to it and run ``ccmake /path/to/SDK``. 

Configure and generate, then
```bash
make
```

In Windows:

Go to SDK folder and execute cmake-gui. Configure and generate for you compiler. If you use Visual Studio, you 
may have problems with the last versions. It has been tested with VS Entreprise 2017, version 15.4.5.
Make sure that you set `CUDA_HOST_COMPILER` variable in cmake-gui, which is your host compiler, and should be something like `C:/Program Files (x86)/Microsoft Visual Studio/2017/Enterprise/VC/Tools/MSVC/14.11.25503/bin/Hostx64/x64/cl.exe`
Then, you can compile it with VS.

### Build Opal

If Optix works, then you can add Opal:
1. Create a subdirectory called `opal` (or whatever you prefer) inside the Optix `SDK` directory.
2. Copy to that subdirectory the files in this repository.
3. Go to the Optix `SDK` and edit the `CMakeList.txt` to add your directory in the list of samples `add_subdirectory(opal)` (use the name of your directory).
3. If you use Optix 5.1 comment out the ``#define OPAL_USE_TRI`` in ``Common.h``. Otherwise, use GeometryTriangles only supported by Optix 6.0
4. Configure and generate again with cmake as above.
5. make (or compile with VS).

In Linux, that should be all, now you should find your `opal` executable and `libopal_s` shared library in the corresponding folders.


In Windows, you will probably need to add a preprocessor definition to compile. In VS, in the Explorer Solution window, right click on the `opal` project, Properties and then go to C/C++ > Preprocessor and edit the preprocessor defintions to add `OPAL_EXPORTS`.

Do this also for `opal_s` but in this case you should add `opal_s_EXPORTS`. 

Make sure that you do that for both Debug and Release configurations, or any other configurations you use.


## Usage
As a standalone application, you can find an `main` method in `main.cpp` with some tests. Test code can be found in the `tests` folder. You can do your own, include it in your application, and 
so on. Recompile and execute. 

As a library link appropriately and go. 

As a Unity plugin, drop the generated .dll in the Plugins folder and create an opal subdirectory with the .cu and Common.h (see below). If the target platform is Linux, do the same but with the .so.

Note that CUDA programs are compiled in execution time with NVRTC. They are read from the `OPTIX_SAMPLES_SDK_DIR` location as set in the code. See also `sutil.cpp` in the Optix SDK, lines 848 and 335.
In the chosen directory you should put all the .cu files and the `Common.h` file. They can be changed without recompiling all the code.
Any other change made in the .cu files is ignored unless copied to that location.
When building an executable with Unity, you have either to use a specific script to create the cudaDir and copy the .cu and .h files to it, or just create and copy manually after the build is done. In the end, along with the Unity executable and files you need to have this specific folder and files 

### Usage remarks
* Avoid placing receivers so that the radius of the receiver sphere overlaps walls or other interacting elements. Depending on the particular case, the result may be incorrect. This is specially important if you use penetration. In general, if the radius do not overlap, the 
result will be correct. 
* If you do not enable depolarization, the results are correct for horizontal and vertical elements (or just slightly leaning) in the scene, because we are assuming it for the polarization and reflections. In addition, we assume that transmitter and receiver have the same polarization.
* If you enable depolarization, arbitrary LINEAR polarizations for transmitter and receiver and leaning walls and environement elements can be used. It has a performance cost, though.



## Contributing
There are a lot of things that can be improved/optimized and a lot of features that can be added, and we are short of people working on it. Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[MIT](https://choosealicense.com/licenses/mit/)
