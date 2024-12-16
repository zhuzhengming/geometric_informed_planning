# Autopilot C++ API 

The `./lib` directory contains code to compile a shared object allowing to use the Python autopilot code from C++. 

## Build the library
Following options are available (from the `./lib` directory ): 
```bash
make        # Build and test everything 
make shared # Build the autopilot library libautopilot.so
make test   # Build a simple test program and run it
make clean  # Remove all executable files inside ./lib 
```

## Update the library path
The library path needs to be updated (run from `./lib`): 
```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:'$(pwd) >> ~/.bashrc
```

## C++ API
The available methods can be found in `./lib/autopilot.hpp`. 