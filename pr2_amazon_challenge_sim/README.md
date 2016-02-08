# pr2_amazon_challenge_sim #

This package brings up a simulated PR2 with a Kiva shelf, setup for the Amazon Picking Challenge ICRA'15
![pr2_gazebo_kiva.png](https://bitbucket.org/repo/GEndLK/images/1160564541-pr2_gazebo_kiva.png)

### Requirements ###

* Make sure you have **pr2_gazebo** and **gazebo** installed :


```
#!python

sudo apt-get install ros-hydro-pr2-* gazebo ros-hydro-gazebo*
```


### Installation ###

* Clone the repository to your catkin workspace
  
```
#!python

git clone https://bitbucket.org/cvapgoestoseattle/pr2_amazon_challenge_sim
```

* Compile the workspace

```
#!python

cd ~/amazon_challenge_ws
catkin_make
```

### Launching the simulation ###

## Empty shelf ##

To simulate a PR2 and empty Kiva shelf in Gazebo:

```
#!python

roslaunch pr2_amazon_challenge_sim pr2_empty_shelf.launch
```

## Test case 1: cheezit_big_original in bin G and crayola_64_ct in bin I ##


```
#!python

roslaunch pr2_amazon_challenge_sim test_case_1.launch
```