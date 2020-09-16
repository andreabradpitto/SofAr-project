# SofAr-project
Robot arm teleoperation using inertial sensory data

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

All the produced code was made on Ubuntu 16.04, with the CoppeliaSim Edu V4 and ROS kinematic, which is the advised configuration.

### Installing

Obtain a copy of this component 
- If the copy is not inside the ROS workspace source folder, copy the component folder "baxter" from its current position to it, using the graphic interface or the shell command

```sh
mv V-rep /absolute/path/of/catkin/workspace/src/V-rep
```
- Activate the bash scripts

```sh
cd /absolute/path/of/catkin/workspace/src/V-rep
cd src
chmod +x logger_launcher.sh
chmod +x coppelialaucher.sh
```

- Do a catkin make inside the catkin workspace main folder

```sh
cd /absolute/path/of/catkin/workspace/
catkin_make
```


## Running the tests: running the simulation with a DUMMY publisher in order to check whether it’s working.

- Digit "roslaunch V-rep baxter_test.launch" and press enter.
In order to start up a master and all this component involved nodes is simply needed to call the launch file baxter_test.launch by digiting on the shell the command line

```sh
roslaunch V-rep baxter_test.launch
```

By doing this the CoppeliaSim environment will be opened on the correct scene.
Use the user interface to handle the simulation. At this point on the shell a message occurs inviting the user to write a command on it. If you digit ‘help’ a list of all the possible commands is shown. In particular you can:
	-start the simulation (‘start’);
	-set the IMU reference system such that it coincides with the human end effector one (‘calibration’);
	-put the simulation in pause in order to restart from the last configuration you reached (‘pause’);
	-stop the simulation (‘stop’) leading the robot arm to the default configuration;
	-set a default configuration you like for each one of the 7 joints (‘set_default’). In the original default configuration each joint 		 is set to 0.
	-close the user interface, all the running topics and the simulator environment ('exit')

To change the input rate, the shell commands from inside the “V-rep” folder are:


```sh
cd src
gedit publisher_ROS_VREP
```

At line 41 the rate can be changed.

```cpp
ros::Rate loop_rate(110);
```

Then, after saving the file, back again in the shell:

```sh
cd ..
cd ..
cd ..
catkin_make
```

In order to apply the changes.


## Deployment

To deploy to a live system the procedures are identical to the one observed in the test, with the only exception of the roslaunch to activate which is baxter_sim.launch and the fact that the system needs to publish to the simulation component.

```sh
roslaunch V-rep baxter_sim.launch
```


## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 


## Authors

*Elena Merlo* & *Matteo Palmas* - *Initial work* - 


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
