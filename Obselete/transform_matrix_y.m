function [T] = transform_matrix_y(deg, D)

T = [cos(deg2rad(deg)), 0, -sin(deg2rad(deg)), D(1);
    0, 1, 0, D(2);
    sin(deg2rad(deg)), 0, cos(deg2rad(deg)), D(3);
    0, 0, 0, 1];