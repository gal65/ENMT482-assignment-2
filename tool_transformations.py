# Calculate product of inverses for each tool

import numpy as np
from transformation_matrices import transform_rotx
from transformation_matrices import transform_roty
from transformation_matrices import transform_rotz

def grinder_tool():
    # Grinder Tool frame in TCP frame
    TCP_T_GT = transform_rotz(np.deg2rad(50), [0, 0, 0])
    
    # PUSHy bit in Grinder Tool frame
    D_PUSH = [0, 0, 102.82]
    GT_T_PUSH = transform_rotz(0, D_PUSH)

    return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))

def portafilter_tool():
    # Portafilter Tool frame in TCP frame
    #TCP_T_PT = transform_rotz(np.deg2rad(50), [0, 0, 0]) to do

    return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))

def cup_tool():
    # Cup Tool frame in TCP frame
    # TCP_T_CT = transform_rotz(np.deg2rad(50), [0, 0, 0]) to do 

    return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))

grinder_tool()