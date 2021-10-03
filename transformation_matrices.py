# Functions for performing full frame transformations
# one for rotation about each axis'

import numpy as np

def transform_rotx(rad, translation_vector):
    transform_matrix = [[1, 0, 0, translation_vector[0]],
                        [0, np.cos(rad), -np.sin(rad), translation_vector[1]],
                        [0, np.sin(rad), np.cos(rad), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

def transform_roty(rad, translation_vector):
    transform_matrix = [[np.cos(rad), 0, -np.sin(rad), translation_vector[0]],
                        [0, 1, 0, translation_vector[1]],
                        [np.sin(rad), 0, np.cos(rad), translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix

def transform_rotz(rad, translation_vector):
    transform_matrix = [[np.cos(rad), -np.sin(rad), 0, translation_vector[0]],
                        [np.sin(rad), np.cos(rad), 0, translation_vector[1]],
                        [0, 0, 1, translation_vector[2]],
                        [0, 0, 0, 1]]
    return transform_matrix