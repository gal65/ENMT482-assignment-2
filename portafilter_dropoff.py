# Portafilter movements in Robotk

# Import of functions
import numpy as np
import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox

# Transforms 
def transform_rotx(rad, translation_vector):
    transform_matrix = [[1, 0, 0, translation_vector[0]],
                        [0, np.cos(rad), -np.sin(rad), translation_vector[1]],
                        [0, np.sin(rad), np.cos(rad), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

def transform_roty(rad, translation_vector):
    transform_matrix = [[np.cos(rad), 0, -np.sin(rad), translation_vector[0]],
                        [0, 1, 0, translation_vector[1]],
                        [np.sin(rad), 0, np.cos(rad), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

def transform_rotz(rad, translation_vector):
    transform_matrix = [[np.cos(rad), -np.sin(rad), 0, translation_vector[0]],
                        [np.sin(rad), np.cos(rad), 0, translation_vector[1]],
                        [0, 0, 1, translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix


# Tool position
def grinder_tool(mode):
    # Grinder Tool frame in TCP frame
    TCP_T_GT = transform_rotz(np.deg2rad(50), [0, 0, 0])
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
    TCP_T_PT = transform_rotz(np.deg2rad(50), [0, 0, 0])
    if (mode.lower() == 'pf1'):
        D_PF1 = [4.71, 0, 144.76]
        PT_T_PF1 = transform_roty(np.deg2rad(-7.5), D_PF1)
        return np.matmul(np.linalg.inv(PT_T_PF1), np.linalg.inv(TCP_T_PT)) 
    else:
        D_PF2 = [-32, 0, 27.56]
        PT_T_PF2 = transform_roty(0, D_PF2)
        return np.matmul(np.linalg.inv(PT_T_PF2), np.linalg.inv(TCP_T_PT))         

def cup_tool():
    # Cup Tool frame in TCP frame
    TCP_T_CT = transform_rotz(np.deg2rad(50), [0, 0, 0]) 
    D_CC = [-47, 0, 186.11]
    CT_T_CC = transform_rotz(0, D_CC)
    return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))
    

# Code for Portafilter Working


RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())


# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

# Grinder Dropoff maths

# Grinder location matricies
 
# Grinder position offset


#J_intermediatepoint = [-151.880896, -97.616411, -59.103383, -112.890980, 90.242082, -161.879346]

#T_grinderapproach = rdk.Mat(T_grinderapproach_np.tolist())

RDK.RunProgram("Portafilter Tool Attach (Tool Stand)", True)

RDK.RunProgram("Portafilter Tool Detach (Tool Stand)", True)
