import math
from calculatefk import forward_kinematics


def rotation_z(base_deg):
    theta = math.radians(base_deg)

    c = math.cos(theta)
    s = math.sin(theta)

    # Return your three-row matrix here.
    return [(c, -s, 0), (s, c, 0), (0, 0, 1)]

def rotate_point(matrix, point):
    x, y, z = point

    # Multiply the first row by the point to calculate new_x.
    new_x = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z 
    # Repeat with the second row for new_y.
    new_y = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z 
    # Repeat with the third row for new_z.
    new_z = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z 
    # Return the three new coordinates.
    return new_x, new_y, new_z

def base_transform(base_deg, shoulder_height):
    rotation = rotation_z(base_deg)

    transformation = [(rotation[0][0], rotation[0][1], rotation[0][2], 0), 
                    (rotation[1][0], rotation[1][1], rotation[1][2], 0), 
                    (rotation[2][0], rotation[2][1], rotation[2][2], shoulder_height),
                    (0, 0, 0, 1)]
    
    return transformation

def transform_point(transformation, point):
    x, y, z = point
    w = 1

    # Calculate new_x using row 0 and all four components.
    new_x = transformation[0][0] * x + transformation[0][1] * y + transformation[0][2] * z + transformation[0][3] * w
    # Repeat for new_y using row 1.
    new_y = transformation[1][0] * x + transformation[1][1] * y + transformation[1][2] * z + transformation[1][3] * w
    # Repeat for new_z using row 2. 
    new_z = transformation[2][0] * x + transformation[2][1] * y + transformation[2][2] * z + transformation[2][3] * w

    # Return new_x, new_y, new_z.
    return new_x, new_y, new_z

def rotation_y(angle_deg):
    theta = math.radians(angle_deg)

    c = math.cos(theta)
    s = math.sin(theta)

    # Return the three rows of the matrix above
    return [(c, 0, s), (0, 1, 0), (-s, 0, c)]

def multiply_matrices(matrix_a, matrix_b):
    result = [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0]
    ]

    # Calculate the result's entries.
    for row in range(4):
        for col in range(4):
            # Add a loop over k here.
            for k in range(4):
                result[row][col] += matrix_a[row][k] * matrix_b[k][col]
     
    return result

def shoulder_transform(shoulder_deg):
    rotation = rotation_y(-shoulder_deg)

    # Copy the three rotation rows, adding a fourth entry of 0 to each.
    # Add the final row: (0, 0, 0, 1).
    # Return the matrix.
    transformation = [(rotation[0][0], rotation[0][1], rotation[0][2], 0), 
                        (rotation[1][0], rotation[1][1], rotation[1][2], 0), 
                        (rotation[2][0], rotation[2][1], rotation[2][2], 0),
                        (0, 0, 0, 1)]
    return transformation

def translation_transform(tx, ty, tz):
    # Return the identity matrix with tx, ty, tz in the final column.
    return [(1, 0, 0, tx), (0, 1, 0, ty), (0, 0, 1, tz), (0, 0, 0, 1)]

def forward_kinematics_transform(link1, link2, base_deg, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length):
    base = base_transform(base_deg, shoulder_height)
    shoulder = shoulder_transform(shoulder_deg)
    combined = multiply_matrices(base, shoulder)

    upper_arm = translation_transform(link1, 0, 0)
    elbow_frame = multiply_matrices(combined, upper_arm)

    elbow_rotation = shoulder_transform(elbow_deg)
    forearm_frame = multiply_matrices(elbow_frame, elbow_rotation)

    forearm = translation_transform(link2, 0, 0)
    wrist_frame = multiply_matrices(forearm_frame, forearm)

    wrist_rotation = shoulder_transform(wrist_deg)
    tool_frame = multiply_matrices(wrist_frame, wrist_rotation)

    tool = translation_transform(tool_length, 0, 0)
    gripping_frame = multiply_matrices(tool_frame, tool)

    return gripping_frame

if __name__ == "__main__":
    link1 = 100
    link2 = 80
    shoulder_height = 50
    tool_length = 30

    # Change only these four angles for each test.
    base_deg = 45
    shoulder_deg = 30
    elbow_deg = 20
    wrist_deg = -50

    trig_position = forward_kinematics(link1, link2, base_deg, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length)
    
    gripping_frame = forward_kinematics_transform(link1, link2, base_deg, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length)
    matrix_position = transform_point(gripping_frame, (0, 0, 0))
    
    matches = all(math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9) for a, b in zip(matrix_position, trig_position))

    print("Matrix:", matrix_position)
    print("Trig:", trig_position)
    print("Results match:", matches)