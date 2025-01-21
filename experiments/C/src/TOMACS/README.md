# Compare the performance of zero-delay cycle and microstep-delay cycle approaches

## Requirements
[Lingua Franca](https://github.com/lf-lang/lingua-franca)

[Lingua Franca Runtime Infrastructure (RTI)](https://github.com/lf-lang/reactor-c/tree/main/core/federated/RTI)

## LAN setup

1. Make sure the device on which federated programs run and the device for running the RTI are connected via LAN.

2. Inside the `*.lf` files in the folders `LAN_*`, modify `localhost` to the IP address of the device running the RTI.

3. Execute `./run_LAN_tests.sh`. ㅑt needs to be run from the same folder where the file is located.. The result files must be stored in the folder `Results/LAN_Results`.

4. To generate the graphs, move to `Results/LAN_Results` and execute `LAN_Combine_Results.py` and `*.gnuplot` files.

## WiFi setup

1. Make sure the device on which federated programs run and the device for running the RTI are connected via WiFi.

2. Inside the `*.lf` files in the folders `WiFi*`, modify `localhost` to the IP address of the device running the RTI.

3. Execute `./run_WiFi_tests.sh`. ㅑt needs to be run from the same folder where the file is located.. The result files must be stored in the folder `Results/WiFi_Results`.

4. To generate the graphs, move to `Results/WiFi_Results` and execute `LAN_Combine_Results.py` and `*.gnuplot` files.

## Results

This folder is for storing the results. The `README` demonstrates our experiment results with tables as well as graphs.