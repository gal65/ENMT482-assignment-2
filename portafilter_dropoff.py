# Portafilter movements in RoboDK

# Import of functions
import numpy as np
import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox

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
        GT_T_PUSH = transform_rotz(0, D_PUSH)
        return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))
    else:
        # PULLy bit in Grinder Tool frame
        D_PULL = [-50, 0, 67.06]
        GT_T_PULL = transform_rotz(0, D_PULL)        
        return np.matmul(np.linalg.inv(GT_T_PULL), np.linalg.inv(TCP_T_GT))

def portafilter_tool(mode):
    # Portafilter Tool frame in TCP frame
    TCP_T_PT = transform_rotz(-50, [0, 0, 0])
    if (mode.lower() == 'pf1'):
        D_PF1 = [4.71, 0, 144.76]
        PT_T_PF = transform_roty(-7.5, D_PF1)
    else:
        D_PF2 = [-32, 0, 27.56]
        PT_T_PF = transform_roty(0, D_PF2)
    return np.matmul(np.linalg.inv(PT_T_PF), np.linalg.inv(TCP_T_PT))         

def cup_tool():
    # Cup Tool frame in TCP frame
    TCP_T_CT = transform_rotz(50, [0, 0, 0]) 
    D_CC = [-47, 0, 186.11]
    CT_T_CC = transform_rotz(0, D_CC)
    return np.matmul(np.linalg.inv(GT_T_PUSH), np.linalg.inv(TCP_T_GT))
    
def offset(x,y,z):
    #Creates a translation transformation matrix
    transform = np.array([[1, 0, 0, x],
           [0, 1, 0, y],
           [0, 0, 1, z],
           [0, 0, 0, 1]])
    return transform



# Code for Portafilter Commands

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


'''Maths for Portafilter to grinder interaction'''
# Finding angle z rotation of ginder frame to UR Frame = 135.204 deg z rot
ur_diff_gr_pf2 =  np.array([370.1, -322.5,65.9]) - np.array([482.7, -434.3, 317.3])
theta_gr = np.rad2deg(np.arctan2(ur_diff_gr_pf2[1],ur_diff_gr_pf2[0]))

# Transforms for converting to UR_PF2
UR_T_G = transform_rotz(theta_gr,[482.7,-432.1,316.1])
G_T_PF2 = transform_roty(90 ,[157.61, 0, -250.45])

## IMPORTANT SET OFFSET OF APPROACH (FINE TUNING)
PF2_T_offset = offset(10,0,0)

# Portafilter to Grinder
UR_T_PF2 = np.matmul(UR_T_G, G_T_PF2)
UR_T_PF2_offset = np.matmul(UR_T_PF2, PF2_T_offset)
T_PF2_np = np.matmul(UR_T_PF2_offset, portafilter_tool('pf2'))

# Portafilter Grinder dropoff
# TUNE drop_T offset [-11,0,0]
drop_T = transform_roty(-7.5, [-11,0,0])
drop_np = np.matmul(UR_T_PF2_offset,  drop_T)
T_drop_np = np.matmul(drop_np, portafilter_tool('pf2'))

'''Maths for portafilter to tamper and scraper interactions'''
# Finding tilt in angle on tamper base
ur_diff_tam_b = np.array([600.1, 52.8, 254.5] - np.array([678.4, 70.7, 250.5]))
theta_tam_b = np.rad2deg(np.arctan(ur_diff_tam_b[1]/ur_diff_tam_b[0])) + 270

# Finding tilt in angle on tamper
ur_diff_tam = np.array([582.5,128.9,236]) - np.array([600.1, 52.8, 254.5])
theta_tam = np.rad2deg(np.arctan(ur_diff_tam[2]/ur_diff_tam[0]))
theta_tam = 0
# Defining Individual Transforms
UR_T_TAM0 = transform_rotz(theta_tam_b, [600.1, 52.8, 254.5])
TAM_b_T_TAM = transform_roty(theta_tam, [-80,0,-55])

#TAM_b_T_TAM_offset = transform_rotz(-90, [0,0,0])

UR_T_TAM = np.matmul(UR_T_TAM0, TAM_b_T_TAM)
#UR_T_TAM_offset = np.matmul (UR_T_TAM, TAM_b_T_TAM_offset)

T_TAM_np = np.matmul(UR_T_TAM, portafilter_tool('pf1'))


# Intermediate points to avoid collisions
# Home to toolstand
J_int_tool = [-138.210000, -70.710000, -86.790000, -106.070000, 90.000000, 0.000000]
# Toolstand to Grinder
J_int_gr_app = [-83.570000, -80.360000, -80.360000, -113.650000, 89.500000, -189.640000]




# Convert Matricies to RDK matricies
T_PF2 = rdk.Mat(T_PF2_np.tolist())
T_drop = rdk.Mat(T_drop_np.tolist())
T_TAM = rdk.Mat(T_TAM_np.tolist())




# Run program Module
# Mount portafilter tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Tool Stand)", True)

# Move to Grinder and drop off tool
robot.MoveJ(J_int_gr_app, blocking=True)
robot.MoveL(T_PF2, blocking=True)
robot.MoveL(T_drop, blocking=True)
RDK.RunProgram("Portafilter Tool Detach (Grinder)", True)

# Reattach to portafilter and move to tamper
robot.MoveJ(T_home, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)
robot.MoveL(T_TAM, blocking=True)

sleep(1)
# Put back on rack
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Portafilter Tool Detach (Tool Stand)", True)
