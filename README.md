# myVIRTUOSE-PyBULLET
Integrating the Virtuose robot with PyBullet using Shared Memory

# 

# Acknowledgements
This would not have been possible without the kind contribution by @erwincoumans found at https://github.com/erwincoumans/pybullet_cpp_sharedmemory_example

# Installation
1. Clone this repo
2. Install [Python]([https://docs.python-guide.org/starting/install3/win/#install3-windows](https://www.python.org/ftp/python/3.11.9/python-3.11.9-amd64.exe).
3. Install [Git](https://github.com/git-for-windows/git/releases/download/v2.44.0.windows.1/Git-2.44.0-64-bit.exe)
4. 
5. Verify python installation using
```console
py --version
py -m pip --version
```
4. Install PyBullet using
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

# Notes of caution
- In any loop of sampling feedback from the Virtuose, ensure to include a Sleep() statement.

fin
