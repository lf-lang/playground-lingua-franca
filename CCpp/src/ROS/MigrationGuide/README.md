
The examples here are used in the [ROS 2 migration guide](https://www.lf-lang.org/docs/handbook/ros-migration-guide).

There are two main components here:
- `lf_simple`, which is a generic ROS 2 C++ package that consists of a simple publisher and a simple subscriber.
- `lf-project`, which is a Lingua Franca project that contains the ported version of `lf_simple` to Lingua Franca. To see how `lf_simple` has been ported to `lf-project`, refer to the [ROS 2 migration guide](https://www.lf-lang.org/docs/handbook/ros-migration-guide).

To use the ported `lf-project` example, you would need to build the ROS 2 package in `ros2_simple_package/lf_simple`, make sure that the `lf_simple` ROS 2 package is properly sourced, and build the Lingua Franca program `lf_project/Main.lf`.

## Building and Running the `lf_simple`

1. Install [ROS 2](https://docs.ros.org/en/humble/Installation.html). Make sure that your environment is [configured](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) for ROS 2.
2. Use `colcon` to build `lf_simple`. For example, on Ubuntu, you could do so by:
```bash
cd `ros2_simple_package/lf_simple`
colcon build
```
3. For the purposes of this tutorial, source the appropriate setup file for `lf_simple`. For example, for bash, use:
```
. install/setup.bash
```

4. (Optional) You could verify that `lf_simple` is properly built and installed by running the nodes:
```bash
ros2 run lf_simple talker &
ros2 run lf_simple listener
```

## Building and Running the `lf-project`

Next, we will provide detailed steps to 

1. Install [lfc (Lingua Franca command-line compiler)](https://www.lf-lang.org/docs/handbook/command-line-tools)'s nightly build.

   **Note:** After lfc is installed successfully, add the path to lfc to the environment. Reopen the terminal and enter `lfc --version` to make sure lfc is installed.

2. Since the ported example utilizes the [federated execution](https://www.lf-lang.org/docs/handbook/distributed-execution), you would need to install the [RTI (Run-Time Infrastructure)](https://www.lf-lang.org/docs/handbook/distributed-execution#installation-of-the-rti).

    Note: Make sure that the RTI is installed succesfully by running `which RTI`.

3. Build `lf_project/src/Main.lf`:
    ```bash
    lfc lf_project/Main.lf
    ```

4. You can run the federated program by using the generated bash script in `lf_project/bin`:
    ```bash
    ./lf_project/bin/Main
    ```

## Change Directory

```
git clone https://github.com/lf-lang/examples-lingua-franca.git
cd examples-lingua-franca/CCpp/src/ROS/MigrationGuide
```
If you have already cloned the repository, only changing directory to `MigrationGuide` is required.


## Modify the Executable to Library 
**NOTE** : This step has been completed if cloned from the repo
1. Open CMakeList.txt under the ROS package
2. Modify all add_executable to add_library
```
 add_executable(talker src/publisher_member_function.cpp)
 -> add_library(talker src/publisher_member_function.cpp)
```
```
 add_executable(listener src/subscriber_member_function.cpp)
 -> add_library(listener src/subscriber_member_function.cpp)
```
3. Replace the orignial install commands with the code below 
```
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```
```
ament_export_targets(export_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs) #add all dependecies here

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS talker listener
  EXPORT export_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
```

4. Open the file `lf_simple_package/lf_simple/src/publisher_member_function.cpp`
5. Remove main function shown below:
```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

6. Open the file `lf_simple_package/lf_simple/src/subscriber_member_function.cpp`
7. Remove main function shown below:
```
int main(int argc, char * argv[])
{
  rclcpp::init(lf_simpleargc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## Build Instructions 

### Build ROS2 Library
```
source [path/to/ros2/environment]
cd lf_simple_package && colcon build
```

### Build Lingua Franca program

```
cd .. #Going back to MigrationGuide folder
source lf_simple_package/install/setup.bash
lfc lf-project/src/Main.lf
. lf-project/bin/Main
```

Now you should have the program running with Lingua Franca communication channels!
