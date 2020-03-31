# ParsianStack
parsian-stack implemented on ROS2

## grSim
The simulator grSim was contributed to the SSL community by Parsian. It performs a physical simulation of SSL robots and publishes SSL-Vision network packages.
you can either found it from this [github link](https://github.com/RoboCup-SSL/grSim) or you can have it by updating your git submodules after cloning this repository.
then go according to the [installation file](https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md).
```bash
git submodule update --recursive --init 
```

## ROS2
to use this project you will have to install ROS2 Eloquent you can refer to [ROS2 installation page](https://index.ros.org/doc/ros2/Installation/Eloquent/) in order to do so.
### Note for macOS users
- when running a node you probably get error because of `poco` dylib that cannot be found
  - you should install the exact version of `poco` that ROS2 wants
  - alternatively. find out what is the exact file name that ROS2 is trying to locate go to that file path and rename your file with the name that ROS2 wants
- when running a node you probably get error because of `tinyxml2` dylib that cannot be found
  - you should install the exact version of `tinyxml2` that ROS2 wants
  - alternatively. find out what is the exact file name that ROS2 is trying to locate go to that file path and rename your file with the name that ROS2 wants
- if you need rosdep you should install it with pip3
    ```bash
    python3 -m pip install rosdep
    sudo rosdep init
    rosdep update
    ```
- instal colcon with pip3
    ```bash
    python3 -m pip install colcon-common-extensions
    ```
- ROS2 uses tango-icon-theme for rqt icons that is not installed by defualt you can install it from [tango release page](http://tango.freedesktop.org/releases/) install the latest version of tango-icon-theme from bottom of the page and go according too the installation file.
  - you may need some dependencies before trying to install tango-icon-theme.
    ```bash
    brew install intltool 
  brew install gettext
  brew link gettext --force
  brew install icon-naming-utils
  brew install ImageMagick
    ```
  - you undo gettext linking after the installation.
    ```bash
    brew unlink gettext
    ```
- to fix OpenSplice warning you need to download it from [OpenSplice release page](https://github.com/ADLINK-IST/opensplice/releases) and download the darwin package. then go as follws.
    ```bash
    mkdir -p ~/ros2_OpenSplice
  cd ~/ros2_OpenSplice
  tar xf ~/Downloads/PXXX-VortexOpenSplice-6.9.190925OSS-HDE-x86_64.darwin10_clang-release-installer.tar
    ```
  and add the following to the end of you `.zshrc`
    ```bash
    export OSPL_HOME="absolute/path/to/ros2_OpenSplice/HDE/x86_64.darwin10_clang"
    ```

## Installation
clone the repository.
```bash
git clone https://github.com/kianbehzad/ParsianStack.git 
```
then add the following lin at the end of your `.zshrc` file.
```bash
source path/to/ParsianStack/parsian_ws/src/parsian_util/tools/env.zsh
```
from here on you can build the work-space simply by this command.
```bash
parsian build
```
### Note foe macOS users
you need `Qt4` libraries for this project install it by brew
```bash
brew tap cartr/qt4
brew install qt@4
```
if you have `Qt5` on your system installed you need to unlink it inorder to cmake find the `Qt4`
```bash
brew unlink qt
brew link qt@4
```
if you wanted to have your `Qt5` back just reverse the above commands
```bash
brew unlink qt@4
brew link qt --force
```

## Running nodes
run the provided grsim launch file.
```bash
ros2 launch parsian_util grsim_launch.py
```
