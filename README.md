# tts
A simple ROS implementation of a Text-To-Speech node i C++.
## Getting Started
To get started with using this tts node, it must be included in the following files as shown below. 
include tts in your nodes who are using tts' ```CMakeLists.txt```:
```CMake
find_package(
	...
  	tts
	...
)
```
in your ```package.xml``` as such:
```xml
<build_depend>tts</build_depend>
<exec_depend>tts</exec_depend>
```
and ofcourse in the node using the API.
### Prerequisites
``` pyttsx3 ``` for python 2
### Usage
Here are some examples of how to use the API given.
#### Add string to queue
This method adds ```str``` to the build in queue.
```cpp
tts::addToQueue("Hello",1)
```

#### Remove string from queue
#### Say some string


#### Print current Services



## Future improvement
