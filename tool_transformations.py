# Calculate product of inverses for each tool

import numpy as np
from transformation_matrices import transform_rotx
from transformation_matrices import transform_roty
from transformation_matrices import transform_rotz

def grinder_tool(mode):
    # Grinder Tool frame in TCP frame
    TCP_T_GT = transform_rotz(50, [0, 0, 0])
    if (mode.lower() == 'push'):
        # PUSHy bit in Grinder Tool frame
        D_PUSH = [0, 0, 102.82]
        GT_T_PUSH = transform_rotz(0, D_PUSH)
        return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))
    else:
        # PULLy bit in Grinder Tool frame
        D_PULL = [-50, 0, 67.06]
        GT_T_PULL = transform_rotz(0, D_PULL)        
        return np.matmul(np.linalg.inv(GT_T_PULL), np.linalg.inv(TCP_T_GT))

def portafilter_tool(mode):
    # Portafilter Tool frame in TCP frame
    TCP_T_PT = transform_rotz(50, [0, 0, 0])
    mode = 'PF1'
    if (mode.lower() == 'pf1'):
        D_PF1 = [4.71, 0, 144.76]
        PT_T_PF1 = transform_roty(  7.5, D_PF1)
        return np.matmul(np.linalg.inv(PT_T_PF1), np.linalg.inv(TCP_T_PT)) 
    else:
        D_PF2 = [-32, 0, 27.56]
        PT_T_PF2 = transform_roty(0, D_PF2)
        return np.matmul(np.linalg.inv(PT_T_PF2), np.linalg.inv(TCP_T_PT))         

def cup_tool():
    # Cup Tool frame in TCP frame
    TCP_T_CT = transform_rotz(50, [0, 0, 0]) 
    D_CC = [-47, 0, 186.11]
    CT_T_CC = transform_rotz(0, D_CC)
    return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))

# grinder_tool('PULL')
# portafilter_tool('pf1')
# cup_tool()
