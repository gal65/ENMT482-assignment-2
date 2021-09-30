clc
clear
close all 

% mdl_ur5;
% q0 = [0, 0, 0, 0, 0, 0];
% ur5.teach(q0);

% Coffee Machine frame in UR5 frame
D_CM = [-368.4; -389; 350.6];
UR_T_CM = transform_matrix_z(104.7209, D_CM);

% Button coords in Coffee Machine frame
D_BUT = [50.67; 35.25; -27.89];
CM_T_BUT = transform_matrix_y(90, D_BUT);

% Button coords in UR5 frame (checkpoint)
%UR_T_BUT = UR_T_CM * CM_T_BUT;

% Grinder Tool frame in TCP frame
TCP_T_GT = transform_matrix_z(50, zeros(3,1));

% Pushy bit in Grinder Tool frame
D_PUSH = [0; 0; 102.82];
GT_T_PUSH = transform_matrix_z(0, D_PUSH);

% Pushy Bit coord in TCP frame (checkpoint)
TCP_T_PB = TCP_T_GT * GT_T_PUSH;

UR_T_TCP = UR_T_CM*CM_T_BUT*inv(GT_T_PUSH)*inv(TCP_T_GT);




