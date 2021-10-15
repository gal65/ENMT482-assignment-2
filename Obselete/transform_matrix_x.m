function [T] = transform_matrix_x(deg, D)

T = [1, 0, 0, D(1);
    0, cos(deg2rad(deg)), -sin(deg2rad(deg)), D(2);
    0, sin(deg2rad(deg)), cos(deg2rad(deg)), D(3);
    0, 0, 0, 1];