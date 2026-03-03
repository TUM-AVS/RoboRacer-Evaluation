import cv2
import numpy as np
import matplotlib.pyplot as plt
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

bag_path = Path('/home/johnny/sa_jonathanmohr/bags_sim/pp_jonathan_sim')

sim_pose_x = []
sim_pose_y = []
velocity = []
typestore = get_typestore(Stores.ROS2_FOXY)
with AnyReader([bag_path]) as reader:
    connections = [x for x in reader.connections if x.topic == '/ego_racecar/odom']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        sim_pose_x.append(msg.pose.pose.position.x)
        sim_pose_y.append(msg.pose.pose.position.y)
        velocity.append(msg.twist.twist.linear.x)
        

sim_pose_x = np.array(sim_pose_x)
sim_pose_y = np.array(sim_pose_y)

waypoints = np.genfromtxt('/home/johnny/sa_jonathanmohr/Maps_Racelines/FTM_Halle_Large_Track/mintime_SM_0_05_Laps_1_FTM_Halle_LargeTrack.csv', delimiter=";", comments='#')[:, 1:3]


# Load YAML data
with open('/home/johnny/sa_jonathanmohr/Maps_Racelines/FTM_Halle_Large_Track/FTM_Halle_Large_Track_orig.yaml', 'r') as yaml_file:
    yaml_dict = yaml.full_load(yaml_file)

origin = yaml_dict["origin"]
resolution = float(yaml_dict["resolution"])

# Load the PNG image (replace with your actual image path)
image_path = "/home/johnny/sa_jonathanmohr/Maps_Racelines/FTM_Halle_Large_Track/FTM_Halle_Large_Track_orig.png"
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Find contours
contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
print(type(contours))
print(len(contours))
del contours[-1]
# Transform pixel coordinates to global coordinates
for contour in contours:
    contours_coords_x = []
    contours_coords_y = []
    for point in contour:
        x, y = point[0]
        global_x = origin[0] + x * resolution  # Compute global x coordinate
        global_y = origin[1] + (image.shape[0] - y) * resolution  # Compute global y coordinate
        contours_coords_x.append(global_x)
        contours_coords_y.append(global_y)
    plt.plot(contours_coords_x, contours_coords_y, color='black', linewidth=3)


# Plot 1 Figure
plt.figure(1)
# Plot the sim pose data with the velocities as colormap
sizes = np.ones(len(sim_pose_x))
plt.scatter(sim_pose_x, sim_pose_y, c=velocity, s=sizes, cmap='plasma', label="Driven Trajectory", alpha= 0.7)
plt.colorbar(label='Velocity')  # Add colorbar to show the mapping of speeds to colors
plt.plot(waypoints[:,0], waypoints[:,1], color='red', linewidth=1 , label="Raceline")

# Plot racetrack boundaries
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Sim Odom trajectory with speeds")
plt.legend(loc='lower right')


plt.show()
