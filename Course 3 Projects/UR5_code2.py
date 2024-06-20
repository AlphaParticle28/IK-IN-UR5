import numpy as np
import modern_robotics as mr
import csv

# Given parameters in the problem statement
M01 = np.array([[1, 0, 0, 0], [0, 1, 0, 1], [0, 0, 1, 0.089159], [0, 0, 0, 1]])
M12 = np.array([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]])
M23 = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]])
M34 = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
M45 = np.array([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]])
M56 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]])
M67 = np.array([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]])
G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 0.37, 0.37, 0.37])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 0.8393, 0.8393, 0.8393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 0.2275, 0.2275, 0.2275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 0.1219, 0.1219, 0.1219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 0.1219, 0.1219, 0.1219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.01879, 0.01879, 0.01879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = np.array([[0, 0, 0, 0, 0, 0],
                  [0, 1, 1, 1, 0, 1],
                  [1, 0, 0, 0, -1, 0],
                  [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
                  [0, 0, 0, 0, 0.81725, 0],
                  [0, 0, 0.425, 0.81725, 0, 0.81725]])

# Defining gravity vector
gravity = np.array([0, 0, -9.81])

def simulate_motion(initial_thetas, simulation_time, filename):
    # Small change in time
    dt = 0.01
    # Calculating total number of steps for a given time
    steps = int(simulation_time / dt)
    thetas = np.copy(initial_thetas)
    # Velocity matrix of joints
    dtheta = np.zeros(6)

    joint_angle_config = []
    print(f"Initial joint configuration={thetas}")
    for i in range(steps):
        joint_angle_config.append(thetas.copy())

        # Setting the wrench vector of the manipulator to be 0, as gearing and frictional effects are ignored
        grav = np.zeros(6)
        # Acceleration matrix of the joints
        ddtheta = mr.ForwardDynamics(thetas, dtheta, grav, gravity, np.zeros(6), Mlist, Glist, Slist)

        # Calculating the velocity after finding out the acceleration
        dtheta = dtheta + ddtheta * dt
        # Calculating the joint configuration after finding out the velocity
        thetas = thetas + dtheta * dt
        print(f"Iteration={i + 1}")
        print(f"Joint angles={thetas}")

    # Creation of a .csv file
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(joint_angle_config)

# Scene 1: when the manipulator is at home position and the simulation lasts 3 seconds
theta_list_1 = np.zeros(6)
simulate_motion(theta_list_1, 3, 'simulation1.csv')

# Scene 2: when the 2nd joint of the arm is at -1 radian and the simulation lasts 5 seconds
theta_list_2 = theta_list_1
theta_list_2[1] = -1
simulate_motion(theta_list_2, 5, 'simulation2.csv')