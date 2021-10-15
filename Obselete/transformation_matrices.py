# Functions for performing full frame transformations
# one for rotation about each axis'

import numpy as np

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
