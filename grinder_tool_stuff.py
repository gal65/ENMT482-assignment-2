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

def ultimate_rotation(alpha,beta,gamma,translation_vector):
    transform_matrix = [[np.cos(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                         np.cos(np.deg2rad(alpha))*np.sin(np.deg2rad(beta))*np.sin(np.deg2rad(gamma)) - np.sin(np.deg2rad(alpha))*np.cos(np.deg2rad(gamma)),
                         np.cos(np.deg2rad(alpha))*np.sin(np.deg2rad(beta))*np.cos(np.deg2rad(gamma)) + np.sin(np.deg2rad(alpha))*np.sin(np.deg2rad(gamma)), translation_vector[0]],
                        [np.sin(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                         np.sin(np.deg2rad(alpha))*np.sin(np.deg2rad(beta))*np.sin(np.deg2rad(gamma)) + np.cos(np.deg2rad(alpha))*np.cos(np.deg2rad(gamma)),
                         np.sin(np.deg2rad(alpha))*np.sin(np.deg2rad(beta))*np.cos(np.deg2rad(gamma)) - np.cos(np.deg2rad(alpha))*np.sin(np.deg2rad(gamma)), translation_vector[1]],
                        [-np.sin(np.deg2rad(beta)), np.cos(np.deg2rad(beta))*np.sin(np.deg2rad(gamma)), np.cos(np.deg2rad(beta))*np.cos(np.deg2rad(gamma)), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

# Tool position
def grinder_tool(mode):
    # Grinder Tool frame in TCP frame
    TCP_T_GT = transform_rotz(-50, [0, 0, 0])
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
''' Grinder machine ON button '''
# Finding angle z rotation of grinder frame to UR Frame = 135.204 deg z rot
ur_diff_gr_pf2 =  np.array([370.1, -322.5,65.9]) - np.array([482.7, -434.3, 317.3])
theta_gr = np.rad2deg(np.arctan2(ur_diff_gr_pf2[1],ur_diff_gr_pf2[0]))
# Converting to G_T_BUT
UR_T_GM = transform_rotz(theta_gr,[482.7,-432.1,316.1])
D_BUT1 = [-64.42, 89.82, -227.68]
GM_T_BUT1 = transform_rotx(90, D_BUT1)
GM_T_BUT1 = np.matmul(GM_T_BUT1, transform_roty(18, [0,0,0])) # assuming ON button is on 18 deg angle from y-axis of GM

# Approach offset
BUT1_T_offset = offset(0, 0, -50)
UR_T_BUT1 = np.matmul(UR_T_GM, GM_T_BUT1)
UR_T_BUT1_offset = np.matmul(UR_T_BUT1, BUT1_T_offset)

# Press ON
BUT1_press = offset(0,0,51)

# Final matrix calcs
T_BUT1_np = np.matmul(UR_T_BUT1_offset, gt_push)
GM_ON_np = np.matmul(UR_T_BUT1_offset, BUT1_press)
T_BUT1_press_np = np.matmul(GM_ON_np, gt_push)

''' Grinder machine OFF button '''
# Finding angle z rotation of grinder frame to UR Frame = 135.204 deg z rot
D_BUT2 = [-80.71, 90.26, -227.68]
GM_T_BUT2 = transform_rotx(90, D_BUT2)
GM_T_BUT2 = np.matmul(GM_T_BUT2, transform_roty(5, [-1,0,0])) # assuming OFF button is on 5 deg angle from y-axis of GM

# Approach offset
BUT2_T_offset = offset(0, 0, -51)
UR_T_BUT2 = np.matmul(UR_T_GM, GM_T_BUT2)
UR_T_BUT2_offset = np.matmul(UR_T_BUT2, BUT2_T_offset)

# Press OFF
BUT2_press = offset(0,0,48)

# Final matrix calcs
T_BUT2_np = np.matmul(UR_T_BUT2_offset, gt_push)
GM_OFF_np = np.matmul(UR_T_BUT2_offset, BUT2_press)
T_BUT2_press_np = np.matmul(GM_OFF_np, gt_push)

''' Grinder machine tab '''
D_TAB = np.array([-35.82, 83.8, -153])
GM_T_TAB = np.array([[0, 0, -1, D_TAB[0]],
                             [1, 0, 0, D_TAB[1]],
                             [0, -1, 0, D_TAB[2]],
                             [0, 0, 0, 1]])

''' Angle of face of tab '''
GM_T_TAB = np.matmul(GM_T_TAB, transform_roty(15, [0,0,0])) # could be atan(35.82/83.8) = 23.1 deg
UR_T_TAB = np.matmul(UR_T_GM, GM_T_TAB)

''' TAB OFFSET '''
TAB_OFFSET = offset(0, -20, 0)
UR_T_TAB = np.matmul(UR_T_TAB, TAB_OFFSET)

# Approach offset
approach_offset = offset(40, 0, 40)
UR_T_TAB_offset = np.matmul(UR_T_TAB, approach_offset)

# Pull tab: step 1, prepare to contact
UR_T_TAB_PULL1 = np.matmul(UR_T_TAB, offset(0, 0, 10))

# step 2, first contact
UR_T_TAB_PULL2 = np.matmul(UR_T_TAB, offset(-5,0,0))

# step 4, pull final location
# final location in GM frame, relative to PULL2 position
GM_T_TAB_PULL2 = np.matmul(np.linalg.inv(UR_T_GM),UR_T_TAB_PULL2)
GM_D_PULL2 = np.array(GM_T_TAB_PULL2[:-1,3])
radius = np.sqrt(GM_D_PULL2[0]**2 + GM_D_PULL2[1]**2)
initial_angle = np.arctan2(GM_D_PULL2[0], GM_D_PULL2[1]) # wrt y-axis

''' turning angle '''
turning_angle = 57 # assume 57 degree turn
final_angle = np.deg2rad(turning_angle) - np.abs(initial_angle) # wrt y-axis

y_PULL4 = np.sqrt(radius**2/(1+np.tan(final_angle)**2))
x_PULL4 = np.tan(final_angle) * y_PULL4
GM_D_PULL4 = [x_PULL4, y_PULL4, GM_D_PULL2[2]]

# convert to tab coords
GM_T_TAB_PULL4 = np.array([[0, 0, -1, GM_D_PULL4[0]],
                             [1, 0, 0, GM_D_PULL4[1]],
                             [0, -1, 0, GM_D_PULL4[2]],
                             [0, 0, 0, 1]])
GM_T_TAB_PULL4 = np.matmul(GM_T_TAB_PULL4, transform_roty(15-turning_angle, [0,0,0]))
# convert to UR frame
UR_T_TAB_PULL4 = np.matmul(UR_T_GM, GM_T_TAB_PULL4)

# step 3, intermediate point
# location in GM frame, relative to PULL2 position
final_angle = np.deg2rad(turning_angle/2) - np.abs(initial_angle) # wrt y-axis

y_PULL3 = np.sqrt(radius**2/(1+np.tan(final_angle)**2))
x_PULL3 = np.tan(final_angle) * y_PULL3
GM_D_PULL3 = [x_PULL3, y_PULL3, GM_D_PULL2[2]]

# convert to tab coords
GM_T_TAB_PULL3 = np.array([[0, 0, -1, GM_D_PULL3[0]],
                             [1, 0, 0, GM_D_PULL3[1]],
                             [0, -1, 0, GM_D_PULL3[2]],
                             [0, 0, 0, 1]])
GM_T_TAB_PULL3 = np.matmul(GM_T_TAB_PULL3, transform_roty(15-turning_angle/2, [0,0,0]))
# convert to UR frame
UR_T_TAB_PULL3 = np.matmul(UR_T_GM, GM_T_TAB_PULL3)

# Final matrix calcs
T_TAB_approach_np = np.matmul(UR_T_TAB_offset, gt_pull)
T_TAB_PULL1_np = np.matmul(UR_T_TAB_PULL1, gt_pull)
T_TAB_PULL2_np = np.matmul(UR_T_TAB_PULL2, gt_pull)
T_TAB_PULL3_np = np.matmul(UR_T_TAB_PULL3, gt_pull)
T_TAB_PULL4_np = np.matmul(UR_T_TAB_PULL4, gt_pull)

''' Coffee Machine button'''
D_CM = [-368.4, -389, 350.6]
UR_T_CM = transform_rotz(104.7209, D_CM)
CM_D_BUT = [50.67, 35.25, -27.89]
CM_T_BUT = transform_roty(90, CM_D_BUT)
CM_T_BUT = np.matmul(CM_T_BUT, transform_rotx(-25,[0,0,0]))

# Correction offset for CM ON button
CM_BUT_correction = offset(0,0,0)
CM_T_BUT = np.matmul(CM_T_BUT, CM_BUT_correction)

'''Pressing ON'''
# Define approach
CM_T_BUT3_approach = np.matmul(CM_T_BUT, offset(0, 0, -40))
UR_T_BUT3_approach = np.matmul(UR_T_CM, CM_T_BUT3_approach)
T_BUT3_approach_np = np.matmul(UR_T_BUT3_approach,gt_push)

# press ON
BUT3_press = offset(0,0,43)

# Final matrix calcs
T_BUT3_approach_np = np.matmul(UR_T_BUT3_approach,gt_push)
CM_ON_np = np.matmul(UR_T_BUT3_approach, BUT3_press)
T_BUT3_press_np = np.matmul(CM_ON_np, gt_push)

'''Pressing OFF'''
# Define approach
CM_T_BUT4_approach = np.matmul(CM_T_BUT, offset(-20, 0, -40))
UR_T_BUT4_approach = np.matmul(UR_T_CM, CM_T_BUT4_approach)
T_BUT4_approach_np = np.matmul(UR_T_BUT4_approach,gt_push)

# press OFF
BUT4_press = offset(0,0,43)

# Final matrix calcs
T_BUT4_approach_np = np.matmul(UR_T_BUT4_approach,gt_push)
CM_OFF_np = np.matmul(UR_T_BUT4_approach, BUT4_press)
T_BUT4_press_np = np.matmul(CM_OFF_np, gt_push)

# Convert np matrices to RDK matrices
T_BUT1 = rdk.Mat(T_BUT1_np.tolist())
T_BUT1_press = rdk.Mat(T_BUT1_press_np.tolist())
T_BUT2 = rdk.Mat(T_BUT2_np.tolist())
T_BUT2_press = rdk.Mat(T_BUT2_press_np.tolist())
T_TAB_approach = rdk.Mat(T_TAB_approach_np.tolist())
T_TAB_PULL1 = rdk.Mat(T_TAB_PULL1_np.tolist())
T_TAB_PULL2 = rdk.Mat(T_TAB_PULL2_np.tolist())
T_TAB_PULL3 = rdk.Mat(T_TAB_PULL3_np.tolist())
T_TAB_PULL4 = rdk.Mat(T_TAB_PULL4_np.tolist())
T_BUT3_approach = rdk.Mat(T_BUT3_approach_np.tolist())
T_BUT3_press = rdk.Mat(T_BUT3_press_np.tolist())
T_BUT4_approach = rdk.Mat(T_BUT4_approach_np.tolist())
T_BUT4_press = rdk.Mat(T_BUT4_press_np.tolist())

'''Intermediate points to avoid collisions'''
# Home to toolstand for tool bit change
J_int_tool = [-138.210000, -70.710000, -86.790000, -106.070000, 90.000000, 0.000000]

# Waypoint joint angles
J_int_over_cup = [86.790000, -67.500000, 80.360000, -21.480000, -18.060000, 65.240000]
J_int_BUT = [97.640000, -57.860000, 73.930000, -21.480000, -18.060000, 65.250000]
J_int_TAB = [-61.070000, -106.070000, -96.430000, -154.290000, 260.360000, -132.000000]
J_int_CM = [-150.854031, -103.395493, -125.030703, 48.426196, 140.574931, -40.000000]

# Run program module
'''Grinder tool commands'''

# Attach grinder tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Grinder Tool Attach (Tool Stand)", True)

# Move to grinder machine button
robot.MoveJ(J_int_over_cup, blocking=True)
robot.MoveJ(J_int_BUT, blocking=True)

# Perform ON button press
robot.MoveL(T_BUT1, blocking=True)
robot.MoveL(T_BUT1_press, blocking=True)
sleep(1)
robot.MoveL(T_BUT1, blocking=True)

# Perform OFF button press
robot.MoveL(T_BUT2, blocking=True)
sleep(5) # wait for grinder to finish
robot.MoveL(T_BUT2_press, blocking=True)
sleep(1)
robot.MoveL(T_BUT2, blocking=True)
sleep(1)
robot.MoveJ(J_int_over_cup, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)

# Move to grinder machine tab
robot.MoveJ(J_int_TAB, blocking=True)
robot.MoveL(T_TAB_approach, blocking=True)
robot.MoveL(T_TAB_PULL1, blocking=True)
robot.MoveL(T_TAB_PULL2, blocking=True)
robot.MoveL(T_TAB_PULL3, blocking=True)
robot.MoveL(T_TAB_PULL4, blocking=True)
robot.MoveL(T_TAB_PULL3, blocking=True)
robot.MoveL(T_TAB_PULL2, blocking=True)
robot.MoveL(T_TAB_PULL3, blocking=True)
robot.MoveL(T_TAB_PULL4, blocking=True)
robot.MoveL(T_TAB_PULL3, blocking=True)
robot.MoveL(T_TAB_PULL2, blocking=True)
robot.MoveL(T_TAB_PULL3, blocking=True)
robot.MoveL(T_TAB_PULL4, blocking=True)
robot.MoveL(T_TAB_PULL3, blocking=True)
robot.MoveL(T_TAB_PULL2, blocking=True)
robot.MoveL(T_TAB_PULL1, blocking=True)

# Detach grinder and return home
RDK.RunProgram("Grinder Tool Detach (Tool Stand)", True)
robot.MoveJ(target, blocking=True)

# Attach grinder tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Grinder Tool Attach (Tool Stand)", True)

# Move to coffee machine 
robot.MoveJ(J_int_CM,blocking=True)

# Coffee machine ON
robot.MoveL(T_BUT3_approach, blocking=True)
robot.MoveL(T_BUT3_press, blocking=True)
sleep(1)
robot.MoveL(T_BUT3_approach, blocking=True)

# Coffee machine OFF
robot.MoveL(T_BUT4_approach, blocking=True)
sleep(2)
robot.MoveL(T_BUT4_press, blocking=True)
sleep(1)
robot.MoveL(T_BUT4_approach, blocking=True)

# Detach grinder and return home
RDK.RunProgram("Grinder Tool Detach (Tool Stand)", True)
robot.MoveJ(target, blocking=True)

