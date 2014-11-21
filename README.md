# ROSRV Overview

[ROSRV](http://fsl.cs.illinois.edu/ROSRV) is a runtime verification framework 
for the [Robot Operating System (ROS)](http://www.ros.org/). ROS is an 
open-source framework for robot software development, providing operating 
system-like functionality on a heterogeneous computer cluster. With the wide 
adoption of ROS, its safety and security are becoming an important problem. 

ROSRV integrates seamlessly with ROS. Its two purposes are (1) monitoring 
safety properties and (2) enforcing security policies. Its core is a runtime 
monitoring infrastructure that intercepts, observes and optionally modifies 
messages passing through the system, to check system's runtime behavior against 
user-defined safety properties and perform desired actions. For automatic 
monitor generation out of formal specifications, ROSRV depends on 
[ROSMOP](https://github.com/runtimeverification/rosmop). ROSRV regulates system 
state and execution of commands by enforcing a user-defined access control 
policy to address security concerns.

## Installation

To install and build ROSRV, please refer to [INSTALL.md](INSTALL.md) for 
instructions.

## Usage

Refer to [docs/Usage.md](docs/Usage.md) for detailed instructions on how to use 
ROSRV.
