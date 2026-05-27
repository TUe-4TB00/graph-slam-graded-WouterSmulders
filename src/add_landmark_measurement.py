import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X
from helperfunctions import add_landmark_measurement_from_global

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 
    pose4 = result.atPose2(X(4))
    landmark2 = result.atPoint2(L(2))

    graph = add_landmark_measurement_from_global(
        graph,
        pose_key = X(4),
        pose = pose4,
        landmark_key = L(2),
        landmark_point = landmark2,
        measurement_noise = MEASUREMENT_NOISE,
        add_factor = True
    )
    
    return graph