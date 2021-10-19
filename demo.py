# DEMO SCRIPT

import numpy as np
import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
from time import sleep

# Transforms
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

# Tool transforms (return inverses)
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

def portafilter_tool(mode):
    # Portafilter Tool frame in TCP frame
    TCP_T_PT = transform_rotz(-50, [0, 0, 0])
    if (mode.lower() == 'pf1'):
        D_PF1 = [4.71, 0, 144.76]
        PT_T_PF = transform_roty(7.5, D_PF1)
    else:
        D_PF2 = [-32, 0, 27.56]
        PT_T_PF = transform_roty(0, D_PF2)
    return np.matmul(np.linalg.inv(PT_T_PF), np.linalg.inv(TCP_T_PT))         

def cup_tool():
    # Cup Tool frame in TCP frame
    TCP_T_CT = transform_rotz(-50, [0, 0, 0]) 
    D_CC = [-47, 0, 186.11]
    CT_T_CC = transform_roty(0, D_CC)
    return np.matmul(np.linalg.inv(CT_T_CC), np.linalg.inv(TCP_T_CT))
    
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

# Define tool inverse matricies
pf1 = portafilter_tool('pf1')
pf2 = portafilter_tool('pf2')
gt_push = grinder_tool('push')
gt_pull = grinder_tool('pull')
cup_t = cup_tool();

# Set Home target as RDK matrix object
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

'''Maths for Portafilter to grinder interaction'''
# Finding angle z rotation of ginder frame to UR Frame = 135.204 deg z rot
ur_diff_gr_pf2 =  np.array([370.1, -322.5,65.9]) - np.array([482.7, -434.3, 317.3])
theta_gr = np.rad2deg(np.arctan2(ur_diff_gr_pf2[1],ur_diff_gr_pf2[0]))

# Transforms for converting to UR_PF2
UR_T_G = transform_rotz(theta_gr,[482.7,-432.1,316.1])
G_T_PF2 = transform_roty(90 ,[157.61, 0, -250.45])

## IMPORTANT SET OFFSET OF APPROACH (SET TO 0.5mm to the left)
PF2_T_offset = transform_rotz(0.05,[15,-0.5,0])

# Portafilter to Grinder
UR_T_PF2 = np.matmul(UR_T_G, G_T_PF2)
UR_T_PF2_offset = np.matmul(UR_T_PF2, PF2_T_offset)
T_PF2_np = np.matmul(UR_T_PF2_offset, pf2)

# Portafilter Grinder dropoff offset
drop_T = transform_roty(-7.5, [-15,0,0])

# Final Matrix Calculations
drop_np = np.matmul(UR_T_PF2_offset,  drop_T)
T_drop_np = np.matmul(drop_np, pf2)


'''Maths for portafilter to tamper interactions'''
# Finding tilt in angle on tamper base
ur_diff_tam_b = np.array([600.1, 52.8, 254.5] - np.array([678.4, 70.7, 250.5]))
theta_tam_b = np.rad2deg(np.arctan(ur_diff_tam_b[1]/ur_diff_tam_b[0])) + 270

# Finding tilt in angle on tamper
ur_diff_tam = np.array([582.5,128.9,236]) - np.array([600.1, 52.8, 254.5])
theta_tam = -np.rad2deg(np.arctan2(ur_diff_tam[2], ur_diff_tam[1]))

# Defining Individual Transforms to tamper frame
UR_T_TAMb = transform_rotz(theta_tam_b, [600.1, 52.8, 254.5])
TAM_b_T_TAM = transform_roty(theta_tam, [-80,0,-55])

## Offset of tamper on approach + orientation and press
tam_offset1 = [10, -5, -50]
tam_offset2 = [10, -5, -30]
TAM_T_TAM_offset1 = np.array([[0, 1, 0, tam_offset1[0]],
                             [0, 0, 1, tam_offset1[1]],
                             [1, 0, 0, tam_offset1[2]],
                             [0, 0, 0, 1]])

TAM_T_TAM_offset2 = np.array([[0, 1, 0, tam_offset2[0]],
                             [0, 0, 1, tam_offset2[1]],
                             [1, 0, 0, tam_offset2[2]],
                             [0, 0, 0, 1]])

# Final Matrix Calculations
UR_T_TAM = np.matmul(UR_T_TAMb, TAM_b_T_TAM)
UR_T_TAM_offset1 = np.matmul (UR_T_TAM, TAM_T_TAM_offset1)
UR_T_TAM_offset2 = np.matmul (UR_T_TAM, TAM_T_TAM_offset2)
T_TAM1_np = np.matmul(UR_T_TAM_offset1, pf1)
T_TAM2_np = np.matmul(UR_T_TAM_offset2, pf1)


'''Maths for portafilter to scraper interactions'''
# Defining Individual Transforms to scraper frame
TAM_b_T_SCR = transform_roty(theta_tam, [70,0,-32])

# Defining Offset on scraper start
scr_offset = [0, 60, 5]
TAM_T_SCR_offset1 = np.array([[0, 1, 0, scr_offset[0]],
                             [0, 0, 1, scr_offset[1]],
                             [1, 0, 0, scr_offset[2]],
                             [0, 0, 0, 1]])

TAM_T_SCR_offset2 = np.array([[0, 1, 0, scr_offset[0]],
                             [0, 0, 1, -scr_offset[1]],
                             [1, 0, 0, scr_offset[2]],
                             [0, 0, 0, 1]])

# Final Matrix Calculations
UR_T_SCR = np.matmul(UR_T_TAMb, TAM_b_T_SCR)
UR_T_SCR_offset1 = np.matmul (UR_T_SCR, TAM_T_SCR_offset1)
UR_T_SCR_offset2 = np.matmul (UR_T_SCR, TAM_T_SCR_offset2)
T_SCR1_np = np.matmul(UR_T_SCR_offset1, pf1)
T_SCR2_np = np.matmul(UR_T_SCR_offset2, pf1)

'''Maths for portafilter to group head interactions'''
# Finding rotation of tool stand frame
ur_diff_tool = np.array([-645.7, 78.5, 19.05] - np.array([-556.5, -77.4, 19.05]))
theta_tool = -45 - np.rad2deg(np.arctan(ur_diff_tool[0]/ ur_diff_tool[1]))

# Defining transform frames
UR_T_TOOL = transform_rotz(theta_tool, [-556.5, -77.4, 19.05])
offset_head1 = [0, 0, -20]
offset_head2 = [0, 0, 40]

TOOL_T_TOOL_offset1 = np.array([[0, 0, -1, 14.9 + offset_head1[0]],
                               [0, 1, 0, 64.9 + offset_head1[1]],
                               [1, 0, 0, 214.0 + offset_head1[2]],
                               [0, 0, 0, 1]])

TOOL_T_TOOL_offset2 = np.array([[0, 0, -1, 14.9 + offset_head2[0]],
                               [0, 1, 0, 64.9 + offset_head2[1]],
                               [1, 0, 0, 167.0 + offset_head2[2]],
                               [0, 0, 0, 1]])

UR_T_TOOL_offset1 = np.matmul(UR_T_TOOL, TOOL_T_TOOL_offset1)
UR_T_TOOL_offset2 = np.matmul(UR_T_TOOL, TOOL_T_TOOL_offset2)
T_TOOL_np = np.matmul(UR_T_TOOL_offset1, pf1)
T_TOOL_ON_np = np.matmul(UR_T_TOOL_offset2, pf1)
T_TOOL_ROT = np.matmul(UR_T_TOOL_offset2, transform_rotx(45, [0,0,0]))
T_TOOL_ROT_np = np.matmul(T_TOOL_ROT, pf1)


'''Maths for cup tool interaction'''
# Defining transforms for cup holder frame
UR_T_CUP_b = transform_rotz(0, [-1.1, -600.8, -20])
CUP_b_T_CUP = offset(0, 0, 180)
CUP_b_T_DEL = offset(0, 0, 217)
cup_offset = [2,0,6]
cup_up = [0,0,220]
del_offset = [2,0,85]

CUP_T_CUP_offset = np.array([[0, 1, 0, cup_offset[0]],
                             [0, 0, -1, cup_offset[1]],
                             [-1, 0, 0, cup_offset[2]],
                             [0, 0, 0, 1]])

CUP_T_CUP_up = np.array(    [[0, 1, 0, cup_up[0] + cup_offset[0]],
                             [0, 0, -1, cup_up[1] + cup_offset[1]],
                             [-1, 0, 0, cup_up[2] + cup_offset[2]],
                             [0, 0, 0, 1]])

DEL_T_DEL_offset = np.array([[0, -1, 0, del_offset[0]],
                             [0, 0, -1, del_offset[1]],
                             [1, 0, 0,  del_offset[2]],
                             [0, 0, 0, 1]])


# Cup approach
UR_T_CUP = np.matmul(UR_T_CUP_b, CUP_b_T_CUP)
UR_T_CUP_offset = np.matmul(UR_T_CUP, CUP_T_CUP_offset)
UR_T_CUP_UP = np.matmul(UR_T_CUP, CUP_T_CUP_up)

# Final delivery
UR_T_DEL = np.matmul(UR_T_CUP_b, CUP_b_T_DEL)
UR_T_DEL_offset = np.matmul(UR_T_DEL, DEL_T_DEL_offset)

# Solving using inverse kinematics
T_CUP_UP_np = np.matmul(UR_T_CUP_UP, cup_t)
T_CUP_np = np.matmul(UR_T_CUP_offset, cup_t)
T_CUP_DEL_np = np.matmul(UR_T_DEL_offset, cup_t)

# Defining transforms for coffee machine frame
D_CM = [-368.4, -389, 350.6]
UR_T_CM = transform_rotz(104.7209, D_CM)
coffee_cup_offset = [5, 0, 80]
CM_T_Base_offset = transform_rotz(85,[-12.68+coffee_cup_offset[0],72+coffee_cup_offset[1],-290+coffee_cup_offset[2]])

Base_offset_T_CM_Cup = np.array([[0, 0, 1, 0],
                              [0, -1, 0, 0],
                              [1, 0, 0, 0],
                              [0, 0, 0, 1]])

UR_T_CM_T_Base_offset = np.matmul(UR_T_CM, CM_T_Base_offset)
UR_T_CM_CUP = np.matmul(UR_T_CM_T_Base_offset, Base_offset_T_CM_Cup)
T_CM_CUP_np = np.matmul(UR_T_CM_CUP, cup_t)

'''Maths for grinder tool interactions'''
''' Grinder machine ON button (VERIFIED)'''
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
BUT1_press = offset(0,0,49) # changed from 51 

# Final matrix calcs
T_BUT1_np = np.matmul(UR_T_BUT1_offset, gt_push)
GM_ON_np = np.matmul(UR_T_BUT1_offset, BUT1_press)
T_BUT1_press_np = np.matmul(GM_ON_np, gt_push)

''' Grinder machine OFF button (VERIFIED)'''
# Finding angle z rotation of grinder frame to UR Frame = 135.204 deg z rot
D_BUT2 = [-80.71, 90.26, -227.68]
GM_T_BUT2 = transform_rotx(90, D_BUT2)
GM_T_BUT2 = np.matmul(GM_T_BUT2, transform_roty(5, [-1,0,0])) # assuming OFF button is on 5 deg angle from y-axis of GM

# Approach offset
BUT2_T_offset = offset(0, 0, -51) 
UR_T_BUT2 = np.matmul(UR_T_GM, GM_T_BUT2)
UR_T_BUT2_offset = np.matmul(UR_T_BUT2, BUT2_T_offset)

# Press OFF
BUT2_press = offset(0,0,46) # changed from 48

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
TAB_OFFSET = offset(0, -30, 0)
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
turning_angle = 69 # 63.5 not far enough
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

# Correction offset for CM button
CM_BUT_correction = offset(7,0,0)
CM_T_BUT = np.matmul(CM_T_BUT, CM_BUT_correction)

'''Pressing ON (VERIFIED)'''
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
'''X VALUE CHANGED FROM -20 TO -13, NEEDS CHECKING) '''
CM_T_BUT4_approach = np.matmul(CM_T_BUT, offset(-13, 0, -40)) 
UR_T_BUT4_approach = np.matmul(UR_T_CM, CM_T_BUT4_approach)
T_BUT4_approach_np = np.matmul(UR_T_BUT4_approach,gt_push)

# press OFF
BUT4_press = offset(0,0,43)

# Final matrix calcs
T_BUT4_approach_np = np.matmul(UR_T_BUT4_approach,gt_push)
CM_OFF_np = np.matmul(UR_T_BUT4_approach, BUT4_press)
T_BUT4_press_np = np.matmul(CM_OFF_np, gt_push)

# Convert np matrices to RDK matrices
T_PF2 = rdk.Mat(T_PF2_np.tolist())
T_drop = rdk.Mat(T_drop_np.tolist())
T_TAM1 = rdk.Mat(T_TAM1_np.tolist())
T_TAM2 = rdk.Mat(T_TAM2_np.tolist())
T_SCR1 = rdk.Mat(T_SCR1_np.tolist())
T_SCR2 = rdk.Mat(T_SCR2_np.tolist())
T_TOOL = rdk.Mat(T_TOOL_np.tolist())
T_TOOL_ON = rdk.Mat(T_TOOL_ON_np.tolist())
T_TOOL_ROT = rdk.Mat(T_TOOL_ROT_np.tolist())
T_CUP = rdk.Mat(T_CUP_np.tolist())
T_CUP_UP = rdk.Mat(T_CUP_UP_np.tolist())
T_CM_CUP = rdk.Mat(T_CM_CUP_np.tolist())
T_CUP_DEL = rdk.Mat(T_CUP_DEL_np.tolist())
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

# Waypoint joint angles:

# Home to toolstand for tool bit change
J_int_tool = [-138.210000, -70.710000, -86.790000, -106.070000, 90.000000, 0.000000]

# Toolstand to Grinder
J_int_to_gr = [-83.570000, -80.360000, -80.360000, -113.650000, 89.500000, -189.640000]

# Approach to Grinder
J_int_gr_app1 = [-19.201600, -53.189157, -138.267149, -175.169392, -71.128105, -235.889759]
J_int_gr_app2 = [0.242289, -76.052063, -150.045531, -133.902406, -44.961973, -219.500000]
J_int_gr_back = [-8.399013, -80.379436, -145.968851, -124.333587, -52.953150, -226.274286]

# Approach Scraper
J_int_scr = [1.350562, -84.577835, -133.368424, -134.568357, -103.198280, -232.058700]

# Approach Tamper
J_int_tam_app1 = [30.875870, -87.133813, -147.913060, -117.368034, -73.916930, -235.889759]

# Approach Group head
J_int_head_app1 = [-65.465489, -80.526318, -150.774462, -117.220771, -67.649351, -228.364063]
J_int_head_app2 = [-151.070000, -80.360000, -150.770000, -117.220000, -67.640000, -221.790000]

# Move up and Twist
J_int_head_mount1 = [-137.809262, -78.887219, -137.874629, -129.501480, -33.344834, -231.541661]
J_int_head_mount2 = [-146.344641, -75.999088, -139.664600, -134.898369, -52.748139, -225.746259]
J_int_head_mount3 = [-152.696579, -75.704600, -139.810264, -136.218434, -65.201818, -223.487787]
J_int_head_mount4 = [-158.306490, -76.278528, -139.399804, -136.580987, -75.709086, -221.922486]

# Move to final position  for placement
J_int_porta_final = [-159.722528, -107.893132, -133.980426, -107.883534, -132.790144, -213.003506]

# Move to cup holder
J_int_cup1 = [-134.990000, -54.640000, -84.520000, -220.010000, -62.420000, -39.170000]
J_int_cup2 = [-53.959522, -75.614552, -153.665357, -131.949195, -53.959522, -40.000000]
J_int_cupback = [-53.998442, -53.866255, -111.676612, -194.457133, -53.998442, -218.749995]

# Approach Coffee Machine (cup tool)
J_int_coffee = [-78.786752, -66.395504, -143.802923, -149.801573, -22.257653, -220.000000]

# Ensure clearance over cupstand 
J_int_over_cup = [86.790000, -67.500000, 80.360000, -21.480000, -18.060000, 65.240000]

# General Grinder button approch
J_int_BUT = [97.640000, -57.860000, 73.930000, -21.480000, -18.060000, 65.250000]

# Grinder tab approach
J_int_TAB = [-61.070000, -106.070000, -96.430000, -154.290000, 260.360000, -132.000000]

# Coffee Machine button approach
J_int_CM = [-150.854031, -103.395493, -125.030703, 48.426196, 140.574931, -40.000000]

# Final backoff after coffee delivery
J_int_del = [-43.305999, -47.161810, -126.722605, -186.115585, -43.305999, -220.000000]

''' Run program Module '''
# Start from home
robot.MoveJ(target, blocking=True)

''' Portafilter commands '''

# Mount portafilter tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Tool Stand)", True)

# Move to Grinder and drop off tool, (works from home with filter attached)
robot.MoveJ(J_int_to_gr, blocking=True)
robot.MoveJ(J_int_gr_app1, blocking=True)
robot.MoveJ(J_int_gr_app2, blocking=True)

robot.MoveL(T_PF2, blocking=True)
robot.MoveL(T_drop, blocking=True)

RDK.RunProgram("Portafilter Tool Detach (Grinder)", True)

#Back away from portafilter and return to home
robot.MoveJ(J_int_gr_back, blocking=True)
robot.MoveJ(target, blocking=True)

# Grinder Tool 
# Attach grinder tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Grinder Tool Attach (Tool Stand)", True)

# GRINDER BUTTON PRESSES
# Move to grinder machine button
robot.MoveJ(J_int_over_cup, blocking=True)
robot.MoveJ(J_int_BUT, blocking=True)

# Perform ON button press
robot.MoveL(T_BUT1, blocking=True)
robot.MoveL(T_BUT1_press, blocking=True)
# sleep(1)
robot.MoveL(T_BUT1, blocking=True)

# Perform OFF button press
robot.MoveL(T_BUT2, blocking=True)
sleep(3) # wait for grinder to finish
robot.MoveL(T_BUT2_press, blocking=True)
robot.MoveL(T_BUT2, blocking=True)
robot.MoveJ(J_int_over_cup, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)

# GRINDER TAB PULL
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

# Reattach to portafilter and move away
robot.MoveJ(J_int_gr_back, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)
robot.MoveJ(J_int_gr_app2, blocking=True)

# Approach and use scraper
robot.MoveL(T_SCR1, blocking=True)
robot.MoveL(T_SCR2, blocking=True)

# Approach and use tamper
robot.MoveJ(J_int_tam_app1, blocking=True)
robot.MoveL(T_TAM1, blocking=True)
robot.MoveL(T_TAM2, blocking=True)
robot.MoveL(T_TAM1, blocking=True)
robot.MoveJ(J_int_tam_app1, blocking=True)

# Approach and mount to group head
robot.MoveJ(J_int_head_app1, blocking=True)
robot.MoveJ(J_int_head_app2, blocking=True)
robot.MoveL(T_TOOL, blocking=True)
robot.MoveL(T_TOOL_ON, blocking=True)
robot.MoveL(T_TOOL_ROT, blocking=True)
robot.MoveL(T_TOOL_ON, blocking=True)
robot.MoveL(T_TOOL, blocking=True)

# Pull out portafilter and bring near coffee machine and pause for 5 seconds
robot.MoveJ(J_int_porta_final, blocking=True)
sleep(5)

'''Cup Tool Commands'''
# Attach cup tool
robot.MoveJ(J_int_coffee, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Cup Tool Attach (Stand)", True)

# Pick up cup
robot.MoveJ(J_int_cup1, blocking=True)
robot.MoveJ(J_int_cup2, blocking=True)
RDK.RunProgram("Cup Tool Open", True)
robot.MoveL(T_CUP, blocking=True)
RDK.RunProgram("Cup Tool Close", True)
robot.MoveL(T_CUP_UP, blocking=True)

# Put cup to coffee machine
robot.MoveJ(J_int_cupback, blocking=True)
robot.MoveJ(J_int_coffee, blocking=True)
robot.MoveL(T_CM_CUP, blocking=True)
RDK.RunProgram("Cup Tool Open", True)
robot.MoveJ(J_int_coffee, blocking=True)
robot.MoveJ(J_int_cupback, blocking=True)
RDK.RunProgram("Cup Tool Close", True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Cup Tool Detach (Stand)", True)
#robot.MoveJ(target, blocking=True)

# COFFEE MACHINE BUTTON PRESSES 
# Attach grinder tool
#robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Grinder Tool Attach (Tool Stand)", True)

# Move to coffee machine 
robot.MoveJ(J_int_CM,blocking=True)

# Coffee machine ON
robot.MoveL(T_BUT3_approach, blocking=True)
robot.MoveL(T_BUT3_press, blocking=True)
robot.MoveL(T_BUT3_approach, blocking=True)

# Coffee machine OFF
robot.MoveL(T_BUT4_approach, blocking=True)
sleep(3)
robot.MoveL(T_BUT4_press, blocking=True)
robot.MoveL(T_BUT4_approach, blocking=True)

# Detach grinder tool 
RDK.RunProgram("Grinder Tool Detach (Tool Stand)", True)
#robot.MoveJ(target, blocking=True)

# Final Pickup and Coffee Delivery
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Cup Tool Attach (Stand)", True)
robot.MoveJ(J_int_cupback, blocking=True)
RDK.RunProgram("Cup Tool Open", True)
robot.MoveJ(J_int_coffee, blocking=True)
robot.MoveL(T_CM_CUP, blocking=True)
RDK.RunProgram("Cup Tool Close", True)
robot.MoveJ(J_int_coffee, blocking=True)
robot.MoveJ(J_int_cupback, blocking=True)
robot.MoveL(T_CUP_DEL, blocking=True)
RDK.RunProgram("Cup Tool Open", True)
robot.MoveJ(J_int_del, blocking=True)
robot.MoveJ(J_int_cupback, blocking=True)
RDK.RunProgram("Cup Tool Close", True)
RDK.RunProgram("Cup Tool Detach (Stand)", True)
robot.MoveJ(target, blocking=True)
