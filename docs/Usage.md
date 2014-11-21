# ROSRV Usage

**Note:** This document assumes that you have followed all
the instructions in [../INSTALL.md](../INSTALL.md) and updated
your PATH according to the Prerequisites section in that file.

## Monitoring

ROSRV depends on ROSMOP for monitoring. Please refer to 
https://github.com/runtimeverification/rosmop for more information.

## Access Control

ROSRV enforces access control based on a user-provided specification of access policies as input configuration. On receiving any XMLRPC request from nodes, RVMaster decides whether the request is allowed to go to the ROSMaster according to the specification.
The policies are currently categorized into four different sections. Under each section, the access policy is written as a key followed by an assignment symbol and a list of values.

`[Nodes]`: *key* = node name, *value* = machine identity allowed to create the specified nodes

`[Subscribers]`: *key* = topic name, *value* = node identity allowed to subscribe to the topic

`[Publishers]`: *key* = topic name, *value* = node identity allowed to publish to the topic

`[Commands]`: *key* = command name, *value* = node identity allowed to perform the command

The following is a sample access control policy for LandShark.
- The `[Group]` section defines three groups of IP addresses.
- In the `[Nodes]` section, `default=localhost` means that by default `localhost` is allowed to create a node with any name, and `/landshark_radar=certikos` that the alias `certikos` is allowed to create a node with name `/landshark_radar`.
- In `[Publishers]`, only nodes running on machine `ocu` can publish to topic `/landshark_control/trigger`.
- In `[Commands]`, `getSystemState=localhost certikos ocu` means that nodes running on machines `localhost`, `certikos`, or `ocu` are allowed to send `getSystemState` requests to ROSMaster, and `shutdown=localhost` that only nodes on `localhost` are allowed to `shutdown` other nodes.

```
[Groups]
localhost = 127.0.0.1
certikos = ip1 ip2 ip3 ip4
ocu = ip5 ip6 ip7 ip8
 
[Nodes]
default=localhost
/landshark_radar=certikos
 
[Publishers]
default=localhost certikos
/landshark_control/trigger= ocu
 
[Subscribers]
default = localhost certikos
/landshark/gps = ocu
 
[Commands]
# Commands: full access
getSystemState = localhost certikos ocu
# Commands: limited access
lookupNode = localhost certikos
# Commands: local access only
shutdown = localhost
```
