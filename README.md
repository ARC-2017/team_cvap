# README #

Scripts and rosinstall files for getting things up and running for the amazon challenge

# HOW TO RUN #

To run it, clone the repo to your home directory, go into the *amazon_challenge_install* folder and do:


```
#!bash

  chmod +x ./install_aragorn.sh
  ./install_aragorn.sh
```

# MERGING THE ROSINSTALL FILE #

```
#!bash

  cd catkin_ws/src
  wstool init 
  wstool merge amazon_challenge.rosinstall
  wstool update
```