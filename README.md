mightex_controller_ros
======================

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
roslaunch mightex_controller mightex_controller.launch
```

```shell
rostopic pub -1 /mightex_controller_node/cmd_current mightex_controller/CmdCurrent -- 1 200
rostopic pub -1 /mightex_controller_node/cmd_off mightex_controller/CmdChannel -- 1
```

