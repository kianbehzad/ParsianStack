
# ParsianStack
parsian-stack implemented on ROS2

## grSim
The simulator grSim was contributed to the SSL community by Parsian. It performs a physical simulation of SSL robots and publishes SSL-Vision network packages.
you can either found it from this [github link!](https://github.com/RoboCup-SSL/grSim) or you can have it by updating your git submodules after cloning this repository.
```bash
git submodule update --recursive --init 
```

## ROS2
to use this project you will have to install ROS2 Eloquent you can refer to [ROS2 installation page!](https://index.ros.org/doc/ros2/Installation/Eloquent/) in order to do so.
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
- ROS2 uses tango-icon-theme for rqt icons that is not installed by defualt you can install it from [tango release page!](http://tango.freedesktop.org/releases/) install the latest version of tango-icon-theme from bottom of the page and go according too the installation file.
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
- to fix OpenSplice warning you need to download it from [OpenSplice release page!](https://github.com/ADLINK-IST/opensplice/releases) and download the darwin package. then go as follws.
    ```bash
    mkdir -p ~/ros2_OpenSplice
	cd ~/ros2_OpenSplice
	tar xf ~/Downloads/PXXX-VortexOpenSplice-6.9.190925OSS-HDE-x86_64.darwin10_clang-release-installer.tar
    ```
  and add the following to the end of you `.zshrc` or `.bashrc`
    ```bash
    export OSPL_HOME="absolute/path/to/ros2_OpenSplice/HDE/x86_64.darwin10_clang"
    ```
    
