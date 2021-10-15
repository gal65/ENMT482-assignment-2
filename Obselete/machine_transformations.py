# Transforms for machines/apparatus in the UR5 frame

import numpy as np
from transformation_matrices import transform_rotx
from transformation_matrices import transform_roty
from transformation_matrices import transform_rotz

def espresso():
    # Espresso plate and machine frame in UR5 frame
    # Note: CM for coffee machine
    D_CM = [-368.4; -389; 350.6]
    UR_T_CM = transform_rotz(104.7209, D_CM)
    
def grinder():

def cup_stand():

def tamper_wedge():
