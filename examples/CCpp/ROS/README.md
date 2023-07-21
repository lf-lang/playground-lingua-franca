This folder contains some examples of possible interactions between ROS 2 and Lingua Franca.

To install the latest version of ROS 2, refer to (https://index.ros.org/doc/ros2/Installation/).

The examples have been tested on Foxy on Ubuntu 20.04 LTS, Humble on Ubuntu 22.04 LTS, and Rolling.

To see instructions on how to set up each example, refer to the files themselves. Here is an
overview of the current examples:

- `src/ROSBuiltInSerialization.lf`: A two reactor sender-receiver example that demonstrates the built-in capability to serialize 
  and deserialize ROS 2 messages in federated programs using the CCpp target.
  
- `src/ROSSerialization.lf`: Similar to ROSBuiltInSerialization but the inner-workings of serialization and deserialization are
  exposed in reaction bodies.

