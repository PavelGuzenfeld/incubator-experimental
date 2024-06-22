import numpy as np


def projection_on_perpendicular_to_yaw(north, east, yaw):
    # yaw vector
    yaw_vector = np.array([np.cos(yaw), np.sin(yaw)])

    # perpendicular to yaw vector
    perp_yaw_vector = np.array([-np.sin(yaw), np.cos(yaw)])

    # ned vector
    ned_vector = np.array([north, east])

    # projection onto perpendicular vector
    projection = np.dot(ned_vector, perp_yaw_vector)

    return projection


# example usage
north = 10  # meters
east = 5  # meters
yaw = np.pi / 4  # radians (45 degrees)

print(
    f"Projection on perpendicular to yaw: {projection_on_perpendicular_to_yaw(north, east, yaw)} meters"
)
