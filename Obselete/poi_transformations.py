# Transforms for points of interaction in their respective machine frames
# Includes offset transformation

import numpy as np
from transformation_matrices import transform_rotx
from transformation_matrices import transform_roty
from transformation_matrices import transform_rotz

def offset(x, y, z):
    offset = [[1, 0, 0, x],
              [0, 1, 0, y],
              [0, 0, 1, z],
              [0, 0, 0, 1]]
    return offset

def espresso_button():
    D_BUT = [50.67, 35.25, -27.89]
    CM_T_BUT = transform_roty(90, D_BUT)
    CM_T_BUT_approach = np.matmul(CM_T_BUT, offset(0, 0, 75))

def espresso_cup_place():
    D_CP = [-12.68, 72, -290]
    CM_T_CP = transform_rotz(0, D_CP)
    #CM_T_CP_approach = np.matmul(CM_T_CP, offset(0, 0, 0))

# Already given in UR5 frame
#def grinder_PF2():
    #D_PF2 = [157.61, 0, -250.45]
    #GM_T_PF2 = transform_rotz(0, D_PF2)
    
def grinder_button():
    D_GBUT = [-64.42, 89.82, -227.68]
    GM_T_BUT = transform_rotx(-90, D_GBUT)
    GM_T_BUT_approach = np.matmul(GM_T_BUT, offset(0, 0, 75))

def grinder_tab():
    D_TAB = [-35.82, 83.8, -153]
    GM_T_TAB1 = transform_roty(90, D_TAB)
    GM_T_TAB = np.matmul(GM_T_TAB1, transform_rotx(-25, [0, 0, 0])) # THIS IS A GUESS
    GM_T_TAB_approach = np.matmul(GM_T_TAB, offset(0, 25, 25))
    
#def tamper(): # cursed

#def scraper(): # cursed    
    
def cup_lip():
    D_LIP = [-40, 0, 180]
    CS_T_LIP = transform_rotz(0, D_LIP)
    CS_T_LIP_approach = np.matmul(CS_T_LIP, offset(0, 75, 0))

def cup_stand_top():
    D_TOP = [0, 0, 217]
    CS_T_TOP = transform_rotz(0, D_TOP)