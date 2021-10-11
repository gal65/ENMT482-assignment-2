# Import of functions
import numpy as np
import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
from time import sleep

# Transforms (this is out-dated, rad is now np.deg2rad(deg))
def transform_rotx(deg, translation_vector):
    transform_matrix = [[1, 0, 0, translation_vector[0]],
                        [0, np.cos(np.deg2rad(deg)), -np.sin(np.deg2rad(deg)), translation_vector[1]],
                        [0, np.sin(np.deg2rad(deg)), np.cos(np.deg2rad(deg)), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

def transform_roty(deg, translation_vector):
    transform_matrix = [[np.cos(np.deg2rad(deg)), 0, -np.sin(np.deg2rad(deg)), translation_vector[0]],
                        [0, 1, 0, translation_vector[1]],
                        [np.sin(np.deg2rad(deg)), 0, np.cos(np.deg2rad(deg)), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

def transform_rotz(deg, translation_vector):
    transform_matrix = [[np.cos(np.deg2rad(deg)), -np.sin(np.deg2rad(deg)), 0, translation_vector[0]],
                        [np.sin(np.deg2rad(deg)), np.cos(np.deg2rad(deg)), 0, translation_vector[1]],
                        [0, 0, 1, translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

# Tool position
def grinder_tool(mode):
    # Grinder Tool frame in TCP frame
    TCP_T_GT = transform_rotz(50, [0, 0, 0])
    if (mode.lower() == 'push'):
        # PUSHy bit in Grinder Tool frame
        D_PUSH = [0, 0, 102.82]
        GT_T = transform_rotz(0, D_PUSH)
    else:
        # PULLy bit in Grinder Tool frame
        D_PULL = [-50, 0, 67.06]
        GT_T = transform_rotz(0, D_PULL)        
    return np.matmul(np.linalg.inv(GT_T), np.linalg.inv(TCP_T_GT))

def offset(x,y,z):
    #Creates a translation transformation matrix
    transform = np.array([[1, 0, 0, x],
           [0, 1, 0, y],
           [0, 0, 1, z],
           [0, 0, 0, 1]])
    return transform

# Define robot parameters

RDK = rl.Robolink()
robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
target = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool())

# Define tool matrices
gt_push = grinder_tool('push')
gt_pull = grinder_tool('pull')

# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

'''Maths for grinder tool interactions'''
# Grinder button
# Finding angle z rotation of grinder frame to UR Frame = 135.204 deg z rot
ur_diff_gr_pf2 =  np.array([370.1, -322.5,65.9]) - np.array([482.7, -434.3, 317.3])
theta_gr = np.rad2deg(np.arctan2(ur_diff_gr_pf2[1],ur_diff_gr_pf2[0]))
# Converting to G_T_BUT
UR_T_GM = transform_rotz(theta_gr,[482.7,-432.1,316.1])
D_BUT1 = [-64.42, 89.82, -227.68]
GM_T_BUT1 = transform_rotx(90, D_BUT1)
GM_T_BUT1 = np.matmul(GM_T_BUT1, transform_rotz(10, [0,0,0]))
GM_T_BUT1 = np.matmul(GM_T_BUT1, transform_roty(18, [0,0,0]))

## approach offset
BUT1_T_offset = offset(0, 0, -50)

# Grinder tool to Grinder
UR_T_BUT1 = np.matmul(UR_T_GM, GM_T_BUT1)
UR_T_BUT1_offset = np.matmul(UR_T_BUT1, BUT1_T_offset)

# press ON
BUT1_ON_T = offset(0,0,51)

# Final matrix calcs
T_BUT1_np = np.matmul(UR_T_BUT1_offset, gt_push)
ON_np = np.matmul(UR_T_BUT1_offset, BUT1_ON_T)
T_BUT1_ON_np = np.matmul(ON_np, gt_push)

# Convert np matrices to RDK matrices
T_BUT1 = rdk.Mat(T_BUT1_np.tolist())
T_BUT1_ON = rdk.Mat(T_BUT1_ON_np.tolist())

'''Intermediate points to avoid collisions'''
# Home to toolstand for tool bit change
J_int_tool = [-138.210000, -70.710000, -86.790000, -106.070000, 90.000000, 0.000000]

# Toolstand to Grinder
J_int_to_gr = [-83.570000, -80.360000, -80.360000, -113.650000, 89.500000, -189.640000] # need to change

# Run program module
'''Grinder tool commands'''

# Attach grinder tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Grinder Tool Attach (Tool Stand)", True)

# Move to grinder machine 
# robot.MoveJ(J_int_to_gr, blocking=True)
robot.MoveL(T_BUT1, blocking=True)
sleep(2)

# Perform button press
robot.MoveL(T_BUT1_ON, blocking=True)
sleep(1)

robot.MoveL(T_BUT1, blocking=True)
sleep(2)

# Detach grinder and return home
RDK.RunProgram("Grinder Tool Detach (Tool Stand)", True)
robot.MoveJ(target, blocking=True)
