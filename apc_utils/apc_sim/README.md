# apc_sim
Scripts and launch files for simulating APC'16 in Gazebo.

## Setting poses of shelf/simulated objects in Gazebo

Initial poses of simulated objects are set in the **config/<>.yaml** files.

Each configuration file corresponds to a demo/scenario (e.g. apc_pick_task_simple.yaml) which should be coherent with the corresponding **JSON file** in **apc_bt_launcher/data**.

One can specify which bin the object belongs to and the pose of that object relative to the bin. One can also specify the pose of the shelf/tote in the Baxter base frame.

For more information on how to launch a simulated environment, pelase refer to the [apc_launch package](https://gits-15.sys.kth.se/KTH-APC/apc_launch)
