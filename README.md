# myVIRTUOSE-PyBULLET
Integrating the Virtuose robot with PyBullet using Shared Memory

# 

# Acknowledgements
This would not have been possible without the kind contribution by @erwincoumans found at https://github.com/erwincoumans/pybullet_cpp_sharedmemory_example

# Installation
1. Clone this repo
2. Install [Python](https://docs.python-guide.org/starting/install3/win/#install3-windows) using [Chocolatey](https://community.chocolatey.org/courses/installation/installing?method=installing-chocolatey#cmd).
3. 
4. Copy paste the virtuoseAPI.dll into C:/Windows/System32/ - to ensure the built executable can find it in the path when running.
That's it!

# Building the project
Generate CMake Cache
Build all executables
Preferred naming convention
  "[0-9]*-*/[0-9]*.cpp"

# Notes of caution
- In any loop of sampling feedback from the Virtuose, ensure to include a Sleep() statement.

fin
