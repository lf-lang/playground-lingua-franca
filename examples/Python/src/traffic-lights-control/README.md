Run instructions - execute the following commands:

make clean
lfc src/trafficLights.lf

The recommended way of running the program is via tmux:
bin/trafficLights --tmux

Alternatively, open 7 terminal windows and execute each of these commands in a separate terminal:

fed-gen/trafficLights/bin/RTI -i testRun -n 6 -t
fed-gen/trafficLights/bin/federate__I__0 -i testRun
fed-gen/trafficLights/bin/federate__I__1 -i testRun
fed-gen/trafficLights/bin/federate__I__2 -i testRun
fed-gen/trafficLights/bin/federate__HA -i testRun
fed-gen/trafficLights/bin/federate__AI -i testRun
fed-gen/trafficLights/bin/federate__HOC -i testRun

Human Override commands can be issued by typing allowed commands in the terminal where HOC runs and press enter.