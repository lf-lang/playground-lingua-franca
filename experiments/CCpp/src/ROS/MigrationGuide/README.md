# Replace ROS2 Communication Channels with Lingua Franca

## Prerequisites 

### 1. install Lingua Franca lfc (command line compiler)
```
git clone https://github.com/lf-lang/lingua-franca.git
cd lingua-franca/
./gradlew buildLfc
cd bin
pwd
```

After lfc is built successfully, add the path to lfc shown from the last command to the environment. Reopen the terminal and enter `lfc --version` to make sure lfc is installed.

### 2. install Linuga Franca RTI (runtime infrastructure)

```
cd .. #Going back to the lingua-franca repo folder
git submodule update --init
cd org.lflang/src/lib/c/reactor-c/core/federated/RTI
mkdir build && cd build
cmake ..
make
sudo make install
```
Make sure that RTI has installed succesfully by running `which RTI`

### 3. install ROS2 foxy

Follow the instructions on ROS2 website.

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
