# tts
A simple ROS implementation of a Text-To-Speech node.
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
Python:
``` pyttsx3 ``` for python 2
### Installing
### Running the tests
#### Say some string
#### Add string to queue
#### Remove string from queue
#### Print current Services



## Future improvement
