function [T] = transform_matrix(deg, D)

T = [cos(deg2rad(deg)), -sin(deg2rad(deg)), 0, D(1);
    sin(deg2rad(deg)), cos(deg2rad(deg)), 0, D(2);
    0, 0, 1, D(3);
    0, 0, 0, 1];