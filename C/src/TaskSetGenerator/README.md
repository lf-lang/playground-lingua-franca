# Task Set Generator
This is an LF program for generating task sets to evaluate different schedulers in Lingua Franca. 

This also includes python GUI for easily configuring task sets to be generated. 
Configurations supports sporadic vs. periodic, total execution time, number of tasks, and utilization.

## Prerequisites
Python >= 3.7.13


## How to Install prerequisties
* Set the environment variable LF_PATH to the path where Lingua Franca is installed  
  (You can check by using a command `echo $LF_PATH`)

* Install python libraries
```
pip3 install numpy scipy matplotlib pyqt5
```

for MacOS users, 
```
brew install pyqt
```

## How to use

1. Execute python GUI
```
python3 gui.py
```

2. Select options and click [Run] button.

![gui](https://user-images.githubusercontent.com/43602849/171114948-b6820891-a655-4165-af48-16ada7836900.png)

3. A graph representing total execution time for different number of workers will appear.


![graph](https://user-images.githubusercontent.com/43602849/171115217-db1fec79-c088-4704-82eb-1ba817d519d5.png)
