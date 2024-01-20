import numpy as np
import math
import cv2
 

def rotate_matrix(matrix, axis, angle_deg):
    """
    Rotate a given matrix around a specified axis (X, Y, or Z) by a given angle in degrees.

    :param matrix: The matrix to be rotated.
    :param axis: The axis of rotation ('x', 'y', or 'z').
    :param angle_deg: The rotation angle in degrees.
    :return: The rotated matrix.
    """
    angle_rad = math.radians(angle_deg)

    if axis.lower() == 'x':
        R = np.array([
            [1, 0, 0],
            [0, math.cos(angle_rad), -math.sin(angle_rad)],
            [0, math.sin(angle_rad), math.cos(angle_rad)]
        ])
    elif axis.lower() == 'y':
        R = np.array([
            [math.cos(angle_rad), 0, math.sin(angle_rad)],
            [0, 1, 0],
            [-math.sin(angle_rad), 0, math.cos(angle_rad)]
        ])
    elif axis.lower() == 'z':
        R = np.array([
            [math.cos(angle_rad), -math.sin(angle_rad), 0],
            [math.sin(angle_rad), math.cos(angle_rad), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")

    # Extending matrix if necessary to match dimensions
    if matrix.shape[0] == 3 and len(matrix.shape) == 1:  # For 3D vectors
        extended_matrix, jacobian = cv2.Rodrigues(matrix)
        rotated_matrix = np.matmul(R, extended_matrix[:3])
    elif matrix.shape == (3, 3):  # For 3x3 matrices
        rotated_matrix = np.matmul(R, matrix)
    else:
        raise ValueError("Matrix dimensions not supported.")

    return rotated_matrix
if __name__ == '__main__':
    # Example usage
    original_matrix = np.array([0.0, 0.0, 0.0])  # Example 3D vector
    # Can be 'x', 'y', or 'z'
    angle = 90  # Rotation angle in degrees

    rotated_matrix = rotate_matrix(original_matrix, 'z', angle)
    final_matrix = rotate_matrix(rotated_matrix, 'x', angle)
    print("Original Matrix:", original_matrix)
    print("Rotated Matrix:", rotated_matrix)
    print("Final Matrix:", final_matrix)
