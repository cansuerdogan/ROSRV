# Installing JavaMOP from Binaries

Here are instructions for installing JavaMOP from its binary release
zip archive. Users who checked out the source code should follow the
instructions in [src/README.md](src/README.md) to build JavaMOP.

## Prerequisites

ROSRV currently works with ROS Groovy Galapagos release. For monitoring 
purposes, ROSRV depends on ROSMOP.

1. [Git](http://git-scm.com/book/en/Getting-Started-Installing-Git)
v.1.8 or higher
 * Check Git is installed properly: run `git` from a terminal.
2. [ROS Groovy Galapagos](http://wiki.ros.org/groovy)
3. [ROSMOP](https://github.com/runtimeverification/rosmop)
 * Please check [ROSMOP Installation guide](LINK).

## Install and Build

1. Run 'git clone -- recursive https://github.com/runtimeverification/ROSRV.git' 
to check out the source code from the Github repository, including ROSMOP.

2. Add `<ROSRV_HOME>/bin` to your PATH.

3. Follow the instructions in '<ROSRV_HOME>/rosmop/INSTALL.md'.

4. Run 
 * 'cd <ROSRV_HOME>'
 * 'catkin_make'

5. Make sure the target package builds successfully.

See [docs/Usage.md](docs/Usage.md) for information on how to run ROSRV.
Get help or report problems on
[ROSRV's issues page](https://github.com/runtimeverification/ROSRV/issues).
