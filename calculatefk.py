import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def elbow_position(link1, shoulder_deg):
    shoulder_rad = math.radians(shoulder_deg)

    elbow_reach = link1 * math.cos(shoulder_rad)
    elbow_relative_height = link1 * math.sin(shoulder_rad)

    return elbow_reach, elbow_relative_height


def tip_position(link1, link2, shoulder_deg, elbow_deg, wrist_deg, tool_length):
    shoulder_rad = math.radians(shoulder_deg)
    forearm_rad = math.radians(shoulder_deg + elbow_deg)
    tool_rad = math.radians(shoulder_deg + elbow_deg + wrist_deg)

    reach = link1 * math.cos(shoulder_rad) + link2 * math.cos(forearm_rad) + tool_length * math.cos(tool_rad)
    relative_height = link1 * math.sin(shoulder_rad) + link2 * math.sin(forearm_rad) + tool_length * math.sin(tool_rad)

    return reach, relative_height


def arm_plane_position(link1, link2, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length):
    arm_reach, relative_height = tip_position(link1, link2, shoulder_deg, elbow_deg, wrist_deg, tool_length)

    arm_height = shoulder_height + relative_height

    return arm_reach, arm_height


def forward_kinematics(link1, link2, base_deg, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length):
    arm_reach, arm_height = arm_plane_position(link1, link2, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length)

    base_rad = math.radians(base_deg)

    tip_x = arm_reach * math.cos(base_rad)
    tip_y = arm_reach * math.sin(base_rad)
    tip_z = arm_height

    return tip_x, tip_y, tip_z


def elbow_world_position(link1, base_deg, shoulder_deg, shoulder_height):
    
    elbow_reach, relative_height = elbow_position(link1, shoulder_deg)

    # Convert the base angle to radians.
    base_rad = math.radians(base_deg)


    # Calculate elbow_x and elbow_y from elbow_reach.
    elbow_x = elbow_reach * math.cos(base_rad)

    elbow_y = elbow_reach * math.sin(base_rad)
    # Add shoulder_height to get elbow_z.
    elbow_z = relative_height + shoulder_height 

    # Return all three coordinates.
    return elbow_x, elbow_y, elbow_z




link1 = 100
link2 = 80
tool_length = 30
shoulder_height = 50

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
fig.subplots_adjust(bottom=0.32)

ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")
ax.set_zlabel("Z (mm)")

ax.set_xlim(-220, 220)
ax.set_ylim(-220, 220)
ax.set_zlim(-180, 280)
ax.set_box_aspect((440, 440, 460))
ax.view_init(elev=25, azim=-60)

# Create one line that we will update instead of drawing new lines.
arm_line, = ax.plot([], [], [], marker="o", linewidth=3)

# Each slider has its own small plotting area.
base_slider = Slider(fig.add_axes([0.20, 0.24, 0.65, 0.025]), "Base", 0, 360, valinit=30, valstep=1)
shoulder_slider = Slider(fig.add_axes([0.20, 0.19, 0.65, 0.025]), "Shoulder", -90, 90, valinit=45, valstep=1)
elbow_slider = Slider(fig.add_axes([0.20, 0.14, 0.65, 0.025]), "Elbow", -150, 150, valinit=-60, valstep=1)
wrist_slider = Slider(fig.add_axes([0.20, 0.09, 0.65, 0.025]), "Wrist", -180, 180, valinit=-75, valstep=1)


def update_arm(value):
    base_deg = base_slider.val
    shoulder_deg = shoulder_slider.val
    elbow_deg = elbow_slider.val
    wrist_deg = wrist_slider.val

    shoulder = (0, 0, shoulder_height)
    elbow = elbow_world_position(link1, base_deg, shoulder_deg, shoulder_height)
    wrist = forward_kinematics(link1, link2, base_deg, shoulder_deg, elbow_deg, shoulder_height, 0, 0)
    gripping_point = forward_kinematics(link1, link2, base_deg, shoulder_deg, elbow_deg, shoulder_height, wrist_deg, tool_length)

    arm_line.set_data_3d(
        [shoulder[0], elbow[0], wrist[0], gripping_point[0]],
        [shoulder[1], elbow[1], wrist[1], gripping_point[1]],
        [shoulder[2], elbow[2], wrist[2], gripping_point[2]]
    )

    fig.canvas.draw_idle()


# Run update_arm whenever any slider changes.
base_slider.on_changed(update_arm)
shoulder_slider.on_changed(update_arm)
elbow_slider.on_changed(update_arm)
wrist_slider.on_changed(update_arm)

# Draw the starting pose.
update_arm(None)

plt.show()