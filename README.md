# neo_mpo_moveit2
Moveit packages for all the mpo series having manipulators

### Launch Command

The following command uses the mpo robot urdf available in the [`neo_simulation2`](https://github.com/neobotix/neo_simulation2) package. This can be changed by setting the `description_package` launch argument to the respective package name. The `my_robot` launch argument can be changed to either `mpo_500` or `mpo_700`. Similarly, the `ur_type` launch argument supports `ur10` and `ur5e` arms

`ros2 launch neo_ur_moveit_config neo_ur_moveit.launch.py ur_type:=ur10 my_robot:=mpo_700 use_sim_time:=true use_fake_hardware:=true description_package:=neo_simulation2 prefix:=ur10`

- Make sure to use this command after launching the simulation using the following command.
 
`ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_700 arm_type:=ur10`