# IK-IN-UR5
As a part of my recent course project of modern robotics specialization offered by Coursera I had to apply concepts revolving around forward kinematics into the simulation of inverse kinematics, which involved writing the python code for displaying the feasibility of inverse kinematics for a given configuration of UR5 manipulator.

In a nutshell, the project required me to write a python code for determining the possibility of the existence of an inverse kinematic(IK) solution for a given initial configuration of the robotic manipulator using the Newton-Rhapson method to determine the final error term in the configuration, which tests positive for IK, if it lies below a certain upper limit. 

The code was then saved as a '.csv' file which was later called in a custom-made scene in CoppeliaSim simulator, which displayed the simulation of the UR5 manipulator adhering to the given transformation conditions within 5 iterations.

In order to make sure my algorithm is consistent with the theory, here are a few snippets from the book "Modern Robotics: Mechanics, Planning, and Control" by the course instructors Prof.Kevin Lynch and Prof.Frank Park.

![image](https://github.com/AlphaParticle28/IK-IN-UR5/assets/154257982/81ab6157-3dce-40aa-b89a-b9dece34bff2)

![image](https://github.com/AlphaParticle28/IK-IN-UR5/assets/154257982/02b863eb-b51c-46b5-b67f-e7292b20a5bb)

![image](https://github.com/AlphaParticle28/IK-IN-UR5/assets/154257982/73b712f8-2d42-4ab5-8712-cca0513fe83f)

Following is the output provided by the code, affirming the existence of an IK solution for the guessed value of theta_0

![image](https://github.com/AlphaParticle28/IK-IN-UR5/assets/154257982/d9a79c39-85d9-4363-8437-0fa2d17e9ff2)

![image](https://github.com/AlphaParticle28/IK-IN-UR5/assets/154257982/dcd85739-ed82-4c48-8b67-e2b1f908f76b)

Output produced as a .csv file. 

Note: The ith column corresponds to the ith joint angle configuration. The jth row represents the jth joint configuration. (i ranging from 1 to 6 inclusive and j ranging from 1 to 5 inclusive)

![image](https://github.com/AlphaParticle28/IK-IN-UR5/assets/154257982/0b649ed0-1fcd-4b82-a8cf-f129bcba5d1a)
