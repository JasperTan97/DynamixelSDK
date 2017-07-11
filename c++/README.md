To use DynamixelSDK ver. c++ in Arduino version,

1. Remove "port_handler_linux.cpp" and "port_handler_windows.cpp" in /src/dynamixel_sdk/
2. Remove not related sources
  * CHANGELOG.rst
  * CMakeLists.txt
  * package.xml
3. Change folder name from "c++" to "DynamixelSDK"
4. Put newly made "DynamixelSDK" folder into the libraries/ of Arduino IDE

To use DynamixelSDK ver. c++ in ROS package,

1. Remove "port_handler_arduino.cpp" and "port_handler_windows.cpp" in /src/dynamixel_sdk/
2. Remove not releated sources
  * library.properties
  * keywords.txt
3. (Remove all sources in the parent folder, but actually it doesn't matter.)
4. Put whole package into appropriate location, then catkin make, or sudo apt-get install ros-{version}-dynamixel-sdk
