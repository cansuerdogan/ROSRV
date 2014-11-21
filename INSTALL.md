# Installing ROSRV

Here are instructions for installing and building ROSRV by checking out its 
source code on GitHub.

## Prerequisites

ROSRV currently works with ROS Groovy Galapagos release. For monitoring 
purposes, ROSRV depends on ROSMOP.

1. [Git](http://git-scm.com/book/en/Getting-Started-Installing-Git)
v.1.8 or higher
 * Check Git is installed properly: run `git` from a terminal.
2. [ROS Groovy Galapagos](http://wiki.ros.org/groovy)
3. [ROSMOP](http://fsl.cs.illinois.edu/index.php/ROSMOP)
 * Please check the [ROSMOP Installation guide](rosmop/INSTALL.md).

## Install and Build

1. Run `git clone --recursive https://github.com/runtimeverification/ROSRV.git` 
to check out the source code from the Github repository, including 
[ROSMOP](https://github.com/runtimeverification/rosmop).

2. Add `<ROSRV_HOME>/bin` to your PATH.

3. Run 
 * `cd <ROSRV_HOME>`
 * `catkin_make`

4. Make sure the target package builds successfully.

See [docs/Usage.md](docs/Usage.md) for information on how to run ROSRV.
Get help or report problems on
[ROSRV's issues page](https://github.com/runtimeverification/ROSRV/issues).
