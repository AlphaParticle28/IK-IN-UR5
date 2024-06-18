# IK-IN-UR5
As a part of my recent course project of modern robotics specialization offered by Coursera I had to apply concepts revolving around forward kinematics into the simulation of inverse kinematics, which involved writing the python code for displaying the feasibility of inverse kinematics for a given configuration of UR5 manipulator.
In a nutshell, the project required me to write a python code for determining the possibility of the existence of an inverse kinematic(IK) solution for a given initial configuration of the robotic manipulator using the Newton-Rhapson method to determine the final error term in the configuration, which tests positive for IK, if it lies below a certain upper limit.
`import numpy as np
import modern_robotics as mr
import csv

def IKinBodyIterates(B, M, theta_0, Tsd, e_omega, e_v):
    """Inputs values of Screw Matrix in body frame(B), Initial configuration(M), Initial Values of θ(theta_0),
    and error (e_omega and e_v)"""
    theta = theta_0.copy()
    """Making a copy of the original θ list"""
    theta_iterations = [theta.tolist()]
    angular_error = 0
    linear_error = 0
    n = 5  # Number of required iterations
    for i in range(n):
        Tsb = mr.FKinBody(M, B, theta)
        """Finds the transformation matrix Tsb = e^([B1]θ1)e^([B2]θ2)....e^([Bn]θn)M"""
        Tbd = np.dot(np.linalg.inv(Tsb), Tsd)
        """Transformation to {d} frame wrt {b} frame, therefore, Tbd=(Tsb)^-1*Tsd"""
        Vb_in_se3 = mr.MatrixLog6(Tbd)
        """Calculates [Vb]=log(Tbd), the skey-symmetric matrix of Vb"""
        Vb = mr.se3ToVec(Vb_in_se3)
        """Coverts [Vb] to 6X1 Matrix Vb=[ωb vb]^T"""

        angular_error = np.linalg.norm(Vb[0:3])
        """Vb[1,2,3]=ωb"""
        linear_error = np.linalg.norm(Vb[3:6])
        """Vb[4,5,6]=vb"""
        print(f"Iteration={i+1}")
        print(f"Joint vector={theta}")
        print(f"End effector config=\n{Tsb}")
        print(f"||ωb||={angular_error}")
        print(f"||vb||={linear_error}\n")

        if angular_error < e_omega and linear_error < e_v:
            """Comparing the output error to the threshold error """
            break
        J = mr.JacobianBody(B, theta)
        theta = theta + np.dot(np.linalg.pinv(J), Vb)
        """Update θ=θ+(J^+)Vb"""
        theta_iterations.append(theta.tolist())

    print(theta)
    ans = angular_error < e_omega and linear_error < e_v
    return theta, ans, theta_iterations

"""Input values of UR5 according to example 4.5 from the textbook and course project problem statement"""
W1 = 0.109
W2 = 0.082
H1 = 0.089
H2 = 0.095
L1 = 0.425
L2 = 0.392
"""Values of the above mentioned values are in metres(m)"""
B = np.array([[0, 1, 0, W1 + W1, 0, L1 + L2], [0, 0, 1, H2, -(L1 + L2), 0], [0, 0, 1, H2, -L2, 0],
              [0, 0, 1, H2, 0, 0], [0, -1, 0, -W2, 0, 0], [0, 0, 1, 0, 0, 0]]).T

M = np.array([[-1, 0, 0, L1 + L2], [0, 0, 1, W1 + W2], [0, 1, 0, H1 - H2], [0, 0, 0, 1]])

theta_0 = np.array([0, -np.pi / 2 + 0.85, np.pi / 2, 0, np.pi / 2, 0 + 0.10248])
"""Found the fitting value of Theta_0 using trial and error"""
Tsd = np.array([[0, 1, 0, -0.5], [0, 0, -1, 0.1], [-1, 0, 0, 0.1], [0, 0, 0, 1]])
e_omega = 0.001
e_v = 0.0001

theta_list, ik_possible, theta_iterations = IKinBodyIterates(B, M, theta_0, Tsd, e_omega, e_v)
"""Check for feasibility for the guessed values of Theta_0"""
if ik_possible:
    print("IK solution found")
    print(f"Theta={theta_list}")
else:
    print("IK solution not found")

"""Save the theta values in a CSV file"""
with open('iterates.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    for i, theta in enumerate(theta_iterations):
        writer.writerow(theta)`
