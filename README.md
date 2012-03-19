hpp-model-urdf
========

This package implements a library that allows you to load an hpp-model
robot by parsing an urdf robot model file.

An hpp-model robot allows you to:
- Compute robot dynamics with jrl-dynamics.
- Do motion planning with KiteLab.

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The package depends on several packages which have to be available on
your machine.

 - Libraries:
   - Boost (>=1.48.0)
     Boost Test is used in the test suite
     hpp-model     
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
