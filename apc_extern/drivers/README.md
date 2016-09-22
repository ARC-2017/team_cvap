apc_perception sensor drivers
=============================

Contains:

* `iai_bridge` : forked using git subtree from `https://github.com/code-iai/iai_kinect2`
* `ueye_cam` : forked using git subtree from `https://github.com/anqixu/ueye_cam`

 
## iai_bridge

The drivers for the kinect 2. This requires the manual installation of [libfreenect2](https://github.com/OpenKinect/libfreenect2). Follow the instructions in the `iai_bridge` [readme file] (https://gits-15.sys.kth.se/KTH-APC/apc_perception/tree/master/drivers/iai_kinect2) for installing other required dependencies. 

## ueye_cam

The drivers for the XS uEye cameras. This will be built automatically, nothing to be done. 

Other needed components (not included):
* `CUDA 7.5` : available at `https://developer.nvidia.com/cuda-downloads`
