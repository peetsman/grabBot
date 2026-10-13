# grabBot

A personal robotic arm project using Python and an Arduino UNO R3, with AI integration planned for a later stage.

The target mechanism has base rotation, shoulder rotation, elbow rotation, wrist tilt, and a separate gripper actuator. The current Python model calculates joint and gripping-point positions and displays an interactive 3D arm with joint-angle sliders.

## Run the Python model

Install Python, then run these commands from the project folder:

```powershell
py -m pip install -r requirements.txt
py calculatefk.py
```

Move the Base, Shoulder, Elbow, and Wrist sliders to explore the arm. Dimensions are in millimeters, and angle inputs are in degrees. Slider ranges are exploratory and do not represent verified hardware limits.

## Project files

- `calculatefk.py`: forward kinematics and the interactive Matplotlib model.
- `Hello world/`: existing Arduino and PlatformIO starter project.

## Planned development

- Inverse kinematics.
- Trajectory generation and evaluation.
- Motor selection, CAD, and Arduino control.
- Gripper integration.
- Camera-based object detection for pick-and-place tasks.
