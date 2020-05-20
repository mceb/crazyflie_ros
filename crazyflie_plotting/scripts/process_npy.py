"""
Author: Mikulas Cebecauer
Date: 04/11/19

Describtion:
offline processing data from bag file using numpy & scipy

"""

import numpy as np


def read_data(path):
    # Fetches data from .npy file
    
    return np.load(path)


def get_mean(a):
    # Computes mean
    
    return np.mean(a) 


def get_standard_deviation(a):
    # Computes standard deviation
    
    return np.std(a)


def get_max(a):
    # Maximum value of array
    
    return np.amax(a) 


def get_min(a):
    # Minimum value of array
    
    return np.amin(a) 


def get_standard_error(a):
    # Computes standard error
    
    return (get_standard_deviation(a) / np.sqrt(int(len(a))))


def get_variance(a):
    # Computes variance
    
    return np.var(a)    
    

def get_stat(data_arr):
    # prints and returns basic statistical measures of data

    print('Statistics:')
    
    labels = ['px','py', 'pz', 'ox', 'oy', 'oz', 'ow']
    #print(data_arr)
    for i in range(len(data_arr[:, 0])):
        print(labels[i] + ':')
        print('Mean:', get_mean(data_arr[i, :]))
        print('Std:', get_standard_deviation(data_arr[i, :]))
        print('Ser:', get_standard_error(data_arr[i, :]))
        print('Var:', get_variance(data_arr[i, :]))
        print('Max:', get_max(data_arr[i, :]))
        print('Min:', get_min(data_arr[i, :]))
        
    
    return


if __name__ == '__main__':
    

    path = '/home/mikulas/crazyflie_ws/src/crazyflie_ros/crazyflie_plotting/data/20191227/data-agent5.npy'
    data_arr = read_data(path)
    
    get_stat(data_arr)
    
    
