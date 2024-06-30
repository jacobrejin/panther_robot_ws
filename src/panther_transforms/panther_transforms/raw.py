# THis code was cerated to test the correctness of the uwb baselink transform node.

import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix

# Given data
received_pose = [4.355, 15.174, 0.1489]
received_orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion (x, y, z, w)
# create the transformation matrix from the received pose and orientation
received_transform_matrix = np.array([
    [1.0, 0.0, 0.0, received_pose[0]],
    [0.0, 1.0, 0.0, received_pose[1]],
    [0.0, 0.0, 1.0, received_pose[2]],
    [0.0, 0.0, 0.0, 1.0]
])

rotation_matrix = np.array([
    [9.99528981e-01, -3.06890095e-02, -6.18072155e-22, 0.0],
    [3.06890095e-02, 9.99528981e-01, -5.44358595e-21, 0.0],
    [7.84839293e-22, 5.42205390e-21, 1.00000000e+00, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

offset_matrix= np.array([
    [1.0, 0.0, 0.0, -0.15],
    [0.0, 1.0, 0.0, -0.2],
    [0.0, 0.0, 1.0, -0.354],
    [0.0, 0.0, 0.0, 1.0]
])


# Calculate the new transformation matrix by rotating the offset matrix by the rotation matrix
new_transform_matrix = np.dot(rotation_matrix, offset_matrix)



# print the new transformation matrix
print(new_transform_matrix)
print()

# Calculate the final transformation matrix by multiplying the received transformation matrix with the new transformation matrix
final_transform_matrix = np.dot(received_transform_matrix, new_transform_matrix)

# print the final transformation matrix
print(final_transform_matrix)