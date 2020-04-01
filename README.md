# tts
A simple ROS implementation of a Text-To-Speech node in C++.
## Getting Started
First clone the project into your ROS workspace in ``` src ```, 
then include it in the following files, for each node using this project as shown below.

In ```CMakeLists.txt```:
```CMake
find_package(
	...
  	tts
	...
)
```

In ```package.xml```:
```xml
<build_depend>tts</build_depend>
<exec_depend>tts</exec_depend>
```

## Prerequisites
``` pyttsx3 ``` for python 2.
## Usage
To use the project, one possible solution is include the node in your launch file, which could look like this.
```xml
  <node pkg="sender" name="sender" type="sender" output="screen" launch-prefix="gnome-terminal -e" />
  <node pkg="tts" name="tts" type="tts" output="screen" launch-prefix="gnome-terminal -e"/>
```
Be aware that the tts node must launch wihtin 5 seconds before other nodes are launched.
To use tts in your other nodes these must be initialized like so.
```cpp
    // Initiate the node using tts
    ros::init(argc, argv, "node");
    ros::NodeHandle n;
```
Now tts should be at your disposal.
The API can this be used like in the following example to add 2 services, removing one service, say some service and print a table over the services in the internal queue. Here a message connected to a priority is refered to as a service.

```cpp
    // Add an element to the tts queue with priority 1.
    tts::addToQueue("Hello, I'm a tts node and will say this outloaud", 1);

    // Adding another element to the queue with priority 2.
    tts::addToQueue("I'm just a tts node", 2);

    // Removing the element with the specified message and priority.
    tts::removeFromQueue("I'm just a tts node",2);

    // Place this element in the front of the queue.
    tts::say("I'm gonna say this now");

    // Print a table showing the services' message and priority.
    tts::printSrvs();
```
When using the queue the higher priority has the lower number, thus messages with priority 1 are considered twice as important as messages with priority 2, three times as important as messages with priority 3 etc.
The more important the message is, the more often it is going to be said. Currently the time between messages being said is 3 seconds but can be changed in ´´´tts.h´´´ .
Be aware that more functions than shown above will appear when accessing the tts namepsace, **These are for internal use only and should not be applied in usage of this package!** 
## Future Improvements
In future versions of this package the following implementations will be added 
1. Symmetric queue loop for fully reliable service frequenzy pattern.
2. On the fly calculation of queue element instead of precalculated queue. This is for saving memeory.
3. ROS parameters for constants, such as:
   1. Queue size.
   2. Loop frequency for individual services.
   3. Init timeout, for greater flexibility in initialization.
4. Greater protection of functions not meant to be used such as:
   1. ```bool init().```
   2. ``` inline bool callService(ros::ServiceClient &client, const std::string &data, int priority) ```

