import math


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

print(elbow_angle_to_target(100, 80, 180, 0))   # 0.0
print(elbow_angle_to_target(100, 80, 100, 80))  # Approximately 90.0
print(elbow_angle_to_target(100, 80, 20, 0))    
