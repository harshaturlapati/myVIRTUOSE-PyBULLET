# myVIRTUOSE-PyBULLET
Integrating the Virtuose robot with PyBullet using Shared Memory

# 

# Acknowledgements
This would not have been possible without the kind contribution by @erwincoumans found at https://github.com/erwincoumans/pybullet_cpp_sharedmemory_example

# Installation
1. Clone this repo
2. Install [Python](https://www.python.org/downloads/).
3. 
4. Verify python installation using
```console
py --version
py -m pip --version
```
5. Install PyBullet using
```console
pip install pybullet
```
5.Install Setuptools
```console
python -m pip install -U pip setuptools
```
6. Copy paste the virtuoseAPI.dll into C:/Windows/System32/ - to ensure the built executable can find it in the path when running.
That's it!

# Building the project
Generate CMake Cache
Build all executables
Preferred naming convention
  "[0-9]*-*/[0-9]*.cpp"

# Running Virtuose-PyBullet
1. Run PyBullet shared memory
```console
python -m pybullet_utils.runServer
```
2. Run Visual Studio code.

# Notes of caution
- In any loop of sampling feedback from the Virtuose, ensure to include a Sleep() statement.

fin
