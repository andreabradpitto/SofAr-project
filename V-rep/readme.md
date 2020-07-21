# Simulation node

With this node we are able to receive some data encoding the joint velocities to be applied to the 7 Baxter arm joints, to simulate the motion of this robot arm due to such inputs and to write inside a log file the resulting joint configuration for each one of them. In the software complete version these information are also sent back to one of Mathematic nodes to provide the robot current geometry. 


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

### Prerequisites

All the produced code was made on Ubuntu 16.04, with the CoppeliaSim Edu V4 and ROS kinematic, which is the advised configuration.

### Installing

Obtain a copy of this component 
- Copy the component folder "baxter" from its current position inside the ROS workspace source folder, using the graphic interface or the shell command

```sh
mv baxter /absolute/path/of/catkin/workspace/src/baxter
```

- Do a catkin make inside the catkin workspace main folder

```sh
cd /absolute/path/of/catkin/workspace/
catkin_make
```


## Running the tests: running the simulation with a DUMMY publisher in order to check whether it’s working.

- Digit "roslaunch baxter baxter_sim.launch" and press enter.
In order to start up a master and all this component involved nodes is simply needed to call the launch file baxter_sim.launch by digiting on the shell the command line

```sh
     roslaunch baxter baxter_sim.launch
```

By doing this the CoppeliaSim environment will be opened on the correct scene.

- Use the user interface to handle the simulation.
At this point on the shell a message occurs inviting the user to write a command on it. If you digit ‘help’ a list of all the possible commands is shown. In particular you can: 
	- start the simulation (‘start’); 
	- put it in pause in order to restart from the last configuration you reached (‘pause’); 
	- stop the simulation (‘stop’) leading the robot arm to the default configuration; 
	- set a default configuration you like for each one of the 7 joints (‘set_default’). In the original default configuration each joint is set to 0 and only values belonging to the interval [-1,1] are allowed.
	- close the user interface, all the running topics and the simulator environment ('exit')


To change the input rate, the shell commands from inside the “baxter” folder are:

```sh
 cd src
 gedit publisher_ROS_VREP
```

At line 14 the rate can be changed.

```cpp
   ros::Rate loop_rate(25);
```

Then, after saving the file, back again in the shell:

```sh
   cd ..
   cd ..
   cd ...
   catkin_make
```

In order to apply the changes.


## Deployment

To deploy to a live system the procedures are identical to the one observed in the test, with the only exception of the roslaunch to activate which is *placeholder* and the fact that the system needs to publish to the simulation component. (of course the simulation must be activated before the publisher)

```sh
     roslaunch baxter baxter_sim.launch
```


## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 


## Authors

*Elena Merlo* & *Matteo Palmas* - *Initial work* - 


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.


