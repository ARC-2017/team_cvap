
#APC BT
![Behavior Tree](http://i.imgur.com/v8fXZsD.png)

Includes code related to the high level logic of the system.

## [Wiki](https://gits-15.sys.kth.se/KTH-APC/apc_bt/wiki)
This repo [wiki](https://gits-15.sys.kth.se/KTH-APC/apc_bt/wiki) has information about the high level algorithm that the system should be executing.

## Installation

After forking the repo you must initialize the submodules:

```
$ git submodule init
$ git submodule update
```

Alternatively, use the `--recursive` tag when clonning the fork.

You need to have moveit configured the [baxter way](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial).

### Compiler
There is an [issue](https://github.com/nlohmann/json/pull/212) with gcc 4.8 that prevents compiling the json parser used in this project. So first, make sure you have a more recent version of gcc. In ubuntu this can be achieved by doing
```
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt-get update
$ sudo apt-get install gcc-4.9 g++-4.9
```
and your system should now be correctly configured.

Compile the packages by doing ```$ catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.9 install``` at the root of your workspace.

## Usage

```
$ roslaunch apc_launcher apc.launch
```

The code will apparently hang. This is due to the fact that the behavior trees client is waiting for an input in order to start. Insert any key in the terminal an press enter.

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History

TODO: Write history

## Credits

TODO: Write credits

## License

TODO: Write license
