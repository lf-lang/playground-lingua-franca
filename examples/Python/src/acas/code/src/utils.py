from math import *
import numpy as np

# local imports
from ffnn import *

def parse_nnet_format(path_nnet, act_func):
    """Parse a neural network encoded in the .nnet format, and the associated normalization parameters
    
    Parameters
    ----------
    path_nnet : string
        The path to the .nnet file to be parsed
    act_func : string
        The activation function to be computed by the hidden neurons of the network
        
    Return
    ------
    tuple
        A 2-tuple containing:
            (i) a FFNN object
            (ii) a list of 1D numpy arrays corresponding to normalization parameters:
                1st array <- minimal values of the inputs, 
                2nd array <- maximal values of the inputs,
                3rd array <- mean values of the inputs,
                last array <- ranges of the inputs
    
    """
    topology = []
    weights = []
    biases = []
    with open(path_nnet, 'r') as file:
        # Pass header
        line = file.readline()
        line = file.readline()
        line = file.readline()
        # Read the number of layers
        line = file.readline()
        values = line.split(',')
        n_layers = int(values[0]) + 1 # the layers comprise the input layer, the hidden layers and the output layer
        # Read the topology (number of neurons per layer)
        line = file.readline()
        values = line.split(',')
        topology = [int(values[k]) for k in range(n_layers)]
        # Pass the following line (useless)
        line = file.readline()
        # Read the minimal values of inputs
        line = file.readline()
        values = line.split(',')
        x_min = np.array([float(values[k]) for k in range(topology[0])])
        # Read the maximal values of inputs
        line = file.readline()
        values = line.split(',')
        x_max = np.array([float(values[k]) for k in range(topology[0])])
        # Read the mean values of inputs
        line = file.readline()
        values = line.split(',')
        x_mean = np.array([float(values[k]) for k in range(topology[0])])
        # Read the range of inputs
        line = file.readline()
        values = line.split(',')
        x_range = np.array([float(values[k]) for k in range(topology[0])])
        # Read the weights and biases
        for i in range(n_layers - 1):
            # Read weights connecting layer i to i+1
            W = []
            for j in range(topology[i+1]):
                line = file.readline()
                values = line.split(',')
                W_row = [float(values[k]) for k in range(topology[i])]
                W.append(W_row)
            weights.append(np.array(W))
            # Read biases of layer i+1
            B = []
            for j in range(topology[i+1]):
                line = file.readline()
                values = line.split(',')
                B_row = float(values[0])
                B.append(B_row)
            biases.append(np.array(B))
    return (FFNN(n_layers, topology, weights, biases, act_func), [x_min, x_max, x_mean, x_range])
    
