import math
import matplotlib.pyplot as plt
from calculatefk import elbow_world_position, forward_kinematics

joint_limits = {
    "base": (0, 360),
    "shoulder": (-90, 90),
    "elbow": (-150, 150),
    "wrist": (-180, 180)
}

def base_angle_to_target(target_x, target_y):
    # Calculate the direction in radians.
    base_direction = math.atan2(target_y, target_x)
    # Convert it to degrees.
    base_deg = math.degrees(base_direction)
    # Return the angle.
    return base_deg % 360

def horizontal_distance(target_x, target_y):
    return math.sqrt(target_x ** 2 + target_y ** 2)

def wrist_target(target_x, target_y, target_z, shoulder_height, tool_length, tool_tilt_deg):
    # Calculate the gripping point's horizontal distance from the base.
    wrist_horizontal = horizontal_distance(target_x, target_y)
    # Convert the desired tool tilt to radians.
    tool_tilt_rad = math.radians(tool_tilt_deg)
    # Calculate the tool's horizontal and vertical contributions.
    tool_horizontal = tool_length * math.cos(tool_tilt_rad)
    tool_vertical = tool_length * math.sin(tool_tilt_rad)

    # Subtract those contributions to find the wrist target.
    wrist_reach = wrist_horizontal - tool_horizontal

    wrist_height = target_z - tool_vertical
    # Express its height relative to the shoulder.
    wrist_relative_height =  wrist_height - shoulder_height
    # Return wrist_reach and wrist_relative_height.
    return wrist_reach, wrist_relative_height

def elbow_angle_to_target(link1, link2, wrist_reach, wrist_relative_height):
    if link1 <= 0 or link2 <= 0:
        raise ValueError("Link lengths must be greater than zero.")

    distance = math.hypot(wrist_reach, wrist_relative_height)

    minimum_reach = abs(link1 - link2)
    maximum_reach = link1 + link2
    tolerance = 1e-9 * maximum_reach

    if distance < minimum_reach - tolerance or distance > maximum_reach + tolerance:
        raise ValueError("The wrist target is outside the arm's reachable range.")

    cos_elbow = (distance ** 2 - link1 ** 2 - link2 ** 2) / (2 * link1 * link2)

    # Keep small floating-point rounding errors within acos's valid range.
    cos_elbow = max(-1.0, min(1.0, cos_elbow))

    elbow_rad = math.acos(cos_elbow)
    elbow_deg = math.degrees(elbow_rad)

    return elbow_deg

def shoulder_angle_to_target(link1, link2, wrist_reach, wrist_relative_height, elbow_deg):
    # Convert elbow_deg to radians.
    elbow_rad = math.radians(elbow_deg)

    # Calculate the direction toward the wrist target.
    target_direction = math.atan2(wrist_relative_height, wrist_reach)

    # Calculate offset_horizontal and offset_vertical.
    offset_horizontal = link1 + link2 * math.cos(elbow_rad)

    offset_vertical = link2 * math.sin(elbow_rad)

    # Calculate the offset angle using atan2().
    offset_angle = math.atan2(offset_vertical, offset_horizontal)

    # Subtract the offset from the target direction.
    shoulder_rad = target_direction - offset_angle

    # Convert the shoulder angle to degrees and return it.
    shoulder_deg = math.degrees(shoulder_rad)

    return shoulder_deg

def wrist_angle_to_target(tool_tilt_deg, shoulder_deg, elbow_deg):
    # Subtract the shoulder and elbow contributions from the desired tilt.
    wrist_deg = tool_tilt_deg - shoulder_deg - elbow_deg
    # Return the wrist angle in degrees.
    return wrist_deg

def inverse_kinematics(link1, link2, target_x, target_y, target_z, shoulder_height, tool_length, tool_tilt_deg):
    # Calculate the base angle.
    base_deg = base_angle_to_target(target_x, target_y)
    # Calculate and unpack the wrist target.
    wrist_reach, wrist_relative_height = wrist_target(target_x, target_y, target_z, shoulder_height, tool_length, tool_tilt_deg)
    # Calculate the elbow angle.
    elbow_deg = -elbow_angle_to_target(link1, link2, wrist_reach, wrist_relative_height)
    # Calculate the shoulder angle using that elbow angle.
    shoulder_deg = shoulder_angle_to_target(link1, link2, wrist_reach, wrist_relative_height, elbow_deg)
    # Calculate the wrist angle.
    wrist_deg = wrist_angle_to_target(tool_tilt_deg, shoulder_deg, elbow_deg)
    # Return base_deg, shoulder_deg, elbow_deg, wrist_deg.
    return base_deg, shoulder_deg, elbow_deg, wrist_deg

def angle_within_limits(angle_deg, minimum_deg, maximum_deg):
    return minimum_deg <= angle_deg <= maximum_deg

def check_joint_limits(base_deg, shoulder_deg, elbow_deg, wrist_deg, joint_limits):
    joint_angles = {
        "base": base_deg,
        "shoulder": shoulder_deg,
        "elbow": elbow_deg,
        "wrist": wrist_deg
    }

    failed_joints = []

    for joint_name, angle_deg in joint_angles.items():
        minimum_deg, maximum_deg = joint_limits[joint_name]

        if not angle_within_limits(angle_deg, minimum_deg, maximum_deg):
            failed_joints.append(joint_name)

    limits_ok = len(failed_joints) == 0

    return limits_ok, failed_joints

def above_tabletop(shoulder, elbow, wrist, gripping_point):
    for point in (shoulder, elbow, wrist, gripping_point):
        if point[2] < 0:
            return False

    return True

if __name__ == "__main__":
    
    target_x, target_y, target_z = 100, 0, 20
    shoulder_height = 50
    tool_tilt_deg = -90

    base_deg, shoulder_deg, elbow_deg, wrist_deg = inverse_kinematics(100, 80, target_x, target_y, target_z, shoulder_height, 30, tool_tilt_deg)

    shoulder = (0, 0, shoulder_height)
    elbow = elbow_world_position(100, base_deg, shoulder_deg, shoulder_height)
    wrist = forward_kinematics(100, 80, base_deg, shoulder_deg, elbow_deg, shoulder_height, 0, 0)
    gripping_point = forward_kinematics(100, 80, base_deg, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, 30)

    limits_ok, failed_joints = check_joint_limits(30, 120, -160, -75, joint_limits)

    print("Joint limits passed:", limits_ok)
    print("Failed joints:", failed_joints)
    if limits_ok:
        if above_tabletop(shoulder, elbow, wrist, gripping_point):


            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection="3d")

            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_zlabel("Z (mm)")

            ax.set_xlim(-220, 220)
            ax.set_ylim(-220, 220)
            ax.set_zlim(-180, 280)
            ax.set_box_aspect((440, 440, 460))
            ax.view_init(elev=25, azim=-60)

            ax.plot(
                [shoulder[0], elbow[0], wrist[0], gripping_point[0]],
                [shoulder[1], elbow[1], wrist[1], gripping_point[1]],
                [shoulder[2], elbow[2], wrist[2], gripping_point[2]],
                marker="o",
                linewidth=3,
                label="Arm"
            )

            ax.scatter(target_x, target_y, target_z, marker="x", s=100, color="red", label="Target")
            ax.legend()

            plt.show()
        else:
            print("Pose rejected: part of the arm is below the tabletop.")
    else:
        print("Pose rejected. Joints outside their limits:", failed_joints)