# apc_launch
Launch files for full-system deployment.

## Running pick on the real Baxter
```
roslaunch apc_launch apc.launch demo:=apc_pick_task_simple
```
Where the **demo** parameter specifies which scenario to run and it will oad the correct JSON file in **apc_bt_launcher/data**.

## Usage, command-line arguments

```
roslaunch apc_launch apc.launch pick:=<bool> demo:=<string> sim:=<true/false> gazebo:=<true/false> gazebo_gui:=<true/false> sim_bt:=<true/false> sim_perception:=<true/false> sim_grasping:=<true/false> 
```

The arguments are the following:

* **pick** (**MANDATORY PARAMETER!!!**, bool): set to true for performing the pick task, false for the stowing task.
* **demo** (**MANDATORY PARAMETER!!!**, string): indicates which demo/scenario to run. Loads the corresponding JSON file from bt_launcher/data.
* **sim** (optional, default=false): indicates whether we want a simulated Baxter or not. Apart from loading the JSON file it will also load static TFs of the objects according to the transforms specified in [**apc_sim/config**](https://gits-15.sys.kth.se/KTH-APC/apc_utils/tree/master/apc_sim/config)
* **gazebo** (optional, default=false): indicates whether we want a Gazebo simulation or just Rviz.
* **gazebo_gui** (optional, default=true): enables/disables gazebo gui when simulating.
* **sim_grasping** (optional, default=false): simulates grasping.
* **sim_perception**(optional, default=false): simulates perception (essentially disables the launch file). Useful when simulating Baxter in rviz/gazebo.
* **sim_bt** (optional, default=false): disables the behavior tree.
* **test_bt** (optional, default=false):



## Examples


### Simulate APC pick in Gazebo with kleenex_tissue_box in bin_A
```
roslaunch apc_launch apc.launch pick:=true demo:=apc_pick_task_simple sim:=true gazebo:=true sim_perception:=true 
```
You should get the following:

![alt text](http://i.imgur.com/th0r5YI.png)


### Simulate APC pick in rviz with kleenex_tissue_box in bin_A
```
roslaunch apc_launch apc.launch pick:=true demo:=apc_pick_task_simple sim:=true sim_perception:=true 
```
![alt text](http://i.imgur.com/urZrSax.png)




