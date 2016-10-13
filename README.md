#mightex_controller_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
roslaunch mightex_controller mightex_controller.launch
```

```shell
rostopic pub -1 /mightex_controller_node/cmd_current mightex_controller/CmdCurrent -- 0 200
rostopic pub -1 /mightex_controller_node/cmd_off mightex_controller/CmdChannel -- 0
```

##Mightex Device Python Module

For more information, see [mightex_device_python](https://github.com/janelia-pypi/mightex_device_python)
