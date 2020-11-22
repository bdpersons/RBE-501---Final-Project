# RBE501 - Final Semester Project
#### Team: Pictobot
#### Contributors: Samantha Grillo, Nathan Stallings, Ryan Carnemolla, and Brandon Persons

# Simulation Documentation

#### Description
The simulation demonstrates a 3D model of our robotic manipulator by utilizing a URDF. This URDF was created in SolidWorks and implemented by following tutorials found throughout the internet such as the ROS tutorial found at the following link: http://wiki.ros.org/urdf/Tutorials. This simulation allows the team to demonstrate what the experimental robot was doing while it was completing the light painting on a higher level view. Below are the instructions on how to run the simulation within our repository.

#### Instructions
1. Clone the repository. If you need assistance cloning the repository follow the steps found at this link: https://docs.github.com/en/free-pro-team@latest/github/creating-cloning-and-archiving-repositories/cloning-a-repository.
1. Once the repository is clone, open up MATLAB and change your folder directory to the cloned repository.
1. Open the file labeled "main-sim.m"
1. Click "run" under the editor tab.
1. The simulation with the URDF should show up in a 3D MATLAB graph.

*Note: The URDF has to be plotted everytime, therefore the graph is very jumpy/slow. However, it still traces the trajectory and works very well in the experimentation verison of the code.*

# Experimental Documentation

#### Description
The team was able to contact Kevin Harrington, the Lab Manager at WPI, in order to get access to a 3001 arm from previous years. The robotic arm is a fully 3D printed manipulator controlled using MATLAB and a Nucleo board. The low-level C++ code and high-level MATLAB code communicates through a HID packet protocol. In order to get a communication between the two frameworks, we had to setup status servers that sent positions and velocities back and forth. Furthermore, the arm has three degrees of freedom (DOF), allowing it to work in a spherical workspace within the hardware limitations of the robotic manipulator. This allowed us to take the simulation and recreate it within the real world.

#### Instructions
1. The experimental code and robot operates on Ubuntu 16.0.4, therefore you will need a Ubuntu 16.0.4 computer.
1. Clone the repository. If you need assistance cloning the repository follow the steps found at this link: https://docs.github.com/en/free-pro-team@latest/github/creating-cloning-and-archiving-repositories/cloning-a-repository.
1. Follow the ReadMe for setting up the eclipse code: https://github.com/WPIRoboticsEngineering/RBE3001_nucleo_firmware
1. You will need to utilize this repository for the Nucleo Board (instead of the default repository giving by the instructions): https://github.com/bdpersons/RBE-501-Nucelo
1. After plugging the robot into a USB port 2.0 on the computer and powering the robot with 7.2V with 2 Amp, run the Eclipse code from the Nucleo board to start the code.
1. Once the repository is clone, open up MATLAB and change your folder directory to the cloned repository.
1. Open the file labeled "main.m"
1. Click "run" under the editor tab.
1. The robot will begin moving on its own.
