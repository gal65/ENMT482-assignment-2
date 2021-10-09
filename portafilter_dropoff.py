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
    TCP_T_CT = transform_rotz(50, [0, 0, 0]) 
    D_CC = [-47, 0, 186.11]
    CT_T_CC = transform_rotz(0, D_CC)
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

# Define tool matricies
pf1 = portafilter_tool('pf1')
pf2 = portafilter_tool('pf2')
gt_push = grinder_tool('push')
gt_pull = grinder_tool('pull')
cup_t = cup_tool();





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
T_PF2_np = np.matmul(UR_T_PF2_offset, pf2)

# Portafilter Grinder dropoff offset
drop_T = transform_roty(-7.5, [-11,0,0])

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

# Offset of tapper on approach + orientation
TAM_T_TAM_offset = np.array([[0, 1, 0, 10],
                             [0, 0, 1,-5],
                             [1, 0, 0, -50],
                             [0, 0, 0, 1]])

# Final Matrix Calculations
UR_T_TAM = np.matmul(UR_T_TAMb, TAM_b_T_TAM)
UR_T_TAM_offset = np.matmul (UR_T_TAM, TAM_T_TAM_offset)
T_TAM_np = np.matmul(UR_T_TAM_offset, pf1)



'''Maths for portafilter to scraper interactions'''
# Defining Individual Transforms to scraper frame
TAM_b_T_SCR = transform_roty(theta_tam, [70,0,-32])

# Defining Offset on scraper start
TAM_T_SCR_offset = np.array([[0, 1, 0, 0],
                             [0, 0, 1, 50],
                             [1, 0, 0, -5],
                             [0, 0, 0, 1]])

UR_T_SCR = np.matmul(UR_T_TAMb, TAM_b_T_SCR)
UR_T_SCR_offset = np.matmul (UR_T_SCR, TAM_T_SCR_offset)
T_SCR_np = np.matmul(UR_T_SCR_offset, pf1)


'''Maths for portafilter to group head interactions'''
# Finding rotation of tool stand frame
ur_diff_tool = np.array([-645.7, 78.5, 19.05] - np.array([-556.5, -77.4, 19.05]))
theta_tool = -np.rad2deg(np.arctan2(ur_diff_tam[1], ur_diff_tam[0]))

# Defining transform frames
UR_T_TOOL = transform_rotz(theta_tool, [-556.5, -77.4, 19.05])
offset_tool = [0, 0, -20]
TOOL_T_TOOL_offset = np.array([[1, 0, 0, 14.9 + offset_tool[0]],
                               [0, 1, 0, 64.9 + offset_tool[1]],
                               [0, 0, 1, 167.0 + offset_tool[2]],
                               [0, 0, 0, 1]])

UR_T_TOOL_offset = np.matmul(UR_T_TOOL, TOOL_T_TOOL_offset)
T_TOOL_np = np.matmul(UR_T_TOOL, pf1)


'''Intermediate points to avoid collisions'''

# Home to toolstand for tool bit change
J_int_tool = [-138.210000, -70.710000, -86.790000, -106.070000, 90.000000, 0.000000]

# Toolstand to Grinder
J_int_to_gr = [-83.570000, -80.360000, -80.360000, -113.650000, 89.500000, -189.640000]

# Approach to Grinder
J_int_gr_app1 = [-19.201600, -53.189157, -138.267149, -175.169392, -71.128105, -235.889759]
J_int_gr_app2 = [-29.903868, -78.882506, -136.661916, -152.208894, -93.971698, -218.836334]

# Approach and use Scraper
J_int_scr = [1.350562, -84.577835, -133.368424, -134.568357, -103.198280, -232.058700]

# Approach and use Tamper
J_int_tam_app1 = [30.875870, -87.133813, -147.913060, -117.368034, -73.916930, -235.889759]
J_int_tam_press = [27.417668, -93.659585, -134.449217, -124.422190, -77.345353, -235.422196]

# Approach Group head test
J_int_head_app1 = [-65.465489, -80.526318, -150.774462, -117.220771, -67.649351, -228.364063]
J_int_head_app2 = [-151.070000, -80.360000, -150.770000, -117.220000, -67.640000, -221.790000]

# Convert Matricies to RDK matricies
T_PF2 = rdk.Mat(T_PF2_np.tolist())
T_drop = rdk.Mat(T_drop_np.tolist())
T_TAM = rdk.Mat(T_TAM_np.tolist())
T_SCR = rdk.Mat(T_SCR_np.tolist())
T_TOOL = rdk.Mat(T_TOOL_np.tolist())


# Run program Module
# Mount portafilter tool
robot.MoveJ(T_home, blocking=True)
robot.MoveJ(J_int_tool, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Tool Stand)", True)

# Move to Grinder and drop off tool
robot.MoveJ(J_int_to_gr, blocking=True)
robot.MoveJ(J_int_gr_app1, blocking=True)
robot.MoveJ(J_int_gr_app2, blocking=True)

robot.MoveL(T_PF2, blocking=True)
robot.MoveL(T_drop, blocking=True)
RDK.RunProgram("Portafilter Tool Detach (Grinder)", True)

# Reattach to portafilter and move away
robot.MoveJ(T_home, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)
robot.MoveJ(J_int_gr_app2, blocking=True)

# Approach and use scraper
robot.MoveL(T_SCR, blocking=True)
robot.MoveJ(J_int_scr, blocking=True)

# Approach and use tamper
robot.MoveJ(J_int_tam_app1, blocking=True)
robot.MoveL(T_TAM, blocking=True)
robot.MoveJ(J_int_tam_press, blocking=True)
robot.MoveL(T_TAM, blocking=True)
robot.MoveJ(J_int_tam_app1, blocking=True)

# Approach and use group head
robot.MoveJ(J_int_head_app1, blocking=True)
robot.MoveJ(J_int_head_app2, blocking=True)
robot.MoveL(T_TOOL, blocking=True)
sleep(1)
# Put back on rack
robot.MoveJ(J_int_tool, blocking=True)

RDK.RunProgram("Portafilter Tool Detach (Tool Stand)", True)

