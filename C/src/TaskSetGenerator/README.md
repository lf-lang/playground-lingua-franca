# Task Set Generator
This is an LF program for generating task sets to evaluate different schedulers in Lingua Franca. 

This also includes python GUI for easily configuring task sets to be generated. 
Configurations supports sporadic vs. periodic, total execution time, number of tasks, and utilization.

## Prerequisites
Python >= 3.7.13


## How to Install prerequisties
```
pip3 install numpy scipy matplotlib
```

for MacOS users, 
```
brew install python-tk
```

## How to use

1. Execute python GUI
```
python3 gui.py
```

2. Select options and click [Run] button.

![gui](https://user-images.githubusercontent.com/43602849/171104513-d0e79c52-48f3-425a-87cc-588f335a2ec2.png)

3. A graph representing total execution time for different number of workers will appear.

