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

## Build Instructions 

### Change Directory

```
git clone https://github.com/lf-lang/examples-lingua-franca.git
cd examples-lingua-franca/CCpp/src/ROS/MigrationGuide
```
If you have already cloned the repository, only changing directory to `MigrationGuide` is required.
### Build ROS2 Package 

```
source [path/to/ros2/environment]
cd lf_simple_package && colcon build
```

> **INFO** After ROS2 package is built successfully, we need to delete the `int main()` function in both publisher and subscriber files. Having the main functions in `publisher_member_function.cpp` and `subscriber_member_function.cpp` will lead to redefinition errors when we compile Lingua Franca code. 

1. Open the file `lf_simple_package/lf_simple/src/publisher_member_function.cpp`
2. Remove main function shown below:
```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

3. Open the file `lf_simple_package/lf_simple/src/subscriber_member_function.cpp`
4. Remove main function shown below:
```
int main(int argc, char * argv[])
{
  rclcpp::init(lf_simpleargc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
### Build Lingua Franca program

```
cd .. #Going back to MigrationGuide folder
source lf_simple_package/install/setup.bash
lfc lf-project/src/Main.lf
. lf-project/bin/Main
```

Now you should have the program running with Lingua Franca communication channels!