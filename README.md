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

At the moment, only reflections are computed. In future releases we will add difraction.

It supports simultaneous transmissions from multiple transmitters to multiple receivers.


It can be used as a standalone application or as a Unity plugin. If used as
a Unity plugin, a Unity program will send the meshes and update the transforms 
of the elements of the scene. The Unity related code can be found in 
our   [Veneris repository](https://gitlab.com/esteban.egea/veneris).

For more information visit the [Veneris project website](http://pcacribia.upct.es/veneris).

## Installation

### Requirements
You need a modern NVIDIA GPU and updated driver.  CUDA 9.0 or later. Optix 6.5 or 5.1. gcc or Visual Studio.
CMake 3.10

### Install Optix
Download Optix from the [NVIDIA site](https://developer.nvidia.com/optix) and 
follow instructions to install it.

**Updated to Optix 6.5**
It has been tested with the last Optix version, 6.5, and **the performance on the same hardware has improved remarkably**, even without using RTX cores. With Optix 6.5 
 use CUDA 10.0 or 10.1 and it requires a NVIDIA driver at least R435.80 driver or newer. Follow exactly the same steps as below, but with Optix 6.0 and CUDA 10.0.

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
1. Use always the latest release in this repository. Otherwise we cannot guarantee that this instruccions are up to date.
1. Create a subdirectory called `opal` (or whatever you prefer) inside the Optix `SDK` directory. **If you change the name of the subdirectory modify accordingly the value of baseDir in Opal.cpp in initMembers()**.
2. Copy to that subdirectory the files in this repository.
3. Go to the Optix `SDK` and edit the `CMakeList.txt` to add your directory in the list of samples `add_subdirectory(opal)` (use the name of your directory).
3. If you use Optix 5.1 comment out the ``#define OPAL_USE_TRI`` in ``Common.h``. Otherwise, use GeometryTriangles only supported by Optix 6.5
4. Configure and generate again with cmake as above.
5. make (or compile with VS).

In Linux, that should be all, now you should find your `opal` executable and `libopal_s` shared library in the corresponding folders.


In Windows, you will probably need to add a preprocessor definition to compile. In VS, in the Explorer Solution window, right click on the `opal` project, Properties and then go to C/C++ > Preprocessor and edit the preprocessor defintions to add `OPAL_EXPORTS`.

Do this also for `opal_s` but in this case you should add `opal_s_EXPORTS`. 

Make sure that you do that for both Debug and Release configurations, or any other configurations you use.


## Usage
As a standalone application, you can find an `main` method in `tests.cpp` with some tests. You can do your own, include it in your application, and 
so on. Recompile and execute. 

As a library link appropriately and go. 

As a Unity plugin, in the Plugins folder of your Unity project create an `Opal` subdirectory and drop the generated .dll in the Plugins folder and create an `Opal` subdirectory. If the target platform is Linux, do the same but with the .so. In this case, the .so is in 
the `SDK/lib` folder. In that folder you can also find a `ptx` folder, that you have to copy in the `Opal` plugin folder too. 
 You have to copy the `Common.h` also in the `Opal` folder. 
Finally, put in the `Opal` folder all the `optix` tree (see below).

Note that CUDA programs are compiled at runtime with NVRTC. They are inside the `optix` folder.  
They are read with the `baseDir` variable and `optixPrograms` variables or  from the `OPTIX_SAMPLES_SDK_DIR` location if sutil is used.  See also `sutil.cpp` in the Optix SDK, lines 848 and 335.
They can be changed without recompiling all the code.
With Unity, even recompiling, any  change made in the .cu files is ignored unless copied to the plugin location.
When building an executable with Unity, you have either to use a specific script to create the cudaDir and copy the .cu and .h files to it, or just create and copy manually after the build is done. In the end, along with the Unity executable and files you need to have this specific folder and files 

### Usage remarks
* Take a look at the `main.cpp` and the files in the `tests` folder for examples of use. **That is the first thing you should do. There are alternative ways to set up a simulation and 
different simulation types**.
* Sometimes the simulation enters in an infinite tracing loop due to precision errors in the intersections. Changing the value of the minEpsilon parameter usually solve this issue. We have added a 
preprocessor tag in `Common.h` to use a technique for avoiding self-intersections. But this is going to introduce some small precision errors in the computed electric field, since the ray lengths are modified due to a displacement.
If you do not need very high accuracy (which is the usual case) you can  uncomment it. Otherwise keep it, but then you have to tune minEpsilon, which is not scale-invariant. If your simulation gets stuck in a launch, try uncommenting it.  If it is no longer stuck but you need high precision, comment it again an tune minEpsilon until it is removed.
* Avoid placing receivers so that the radius of the receiver sphere overlaps walls or other interacting elements. Depending on the particular case, the result may be incorrect. This is specially important if you use penetration. In general, if the radius do not overlap, the 
result will be correct. 
* If you do not enable depolarization, the results are correct for horizontal and vertical elements (or just slightly leaning) in the scene, because we are assuming it for the polarization and reflections. In addition, we assume that transmitter and receiver have the same polarization.
* If you enable depolarization, arbitrary LINEAR polarizations for transmitter and receiver and leaning walls and environment elements can be used.
* Diffraction can be used together with any of the reflection methods



## Contributing
There are a lot of things that can be improved/optimized and a lot of features that can be added, and we are short of people working on it. Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[MIT](https://choosealicense.com/licenses/mit/)
