ODE Simulator
=============

## Installation Instructions

1. If not already, install  and initialize ROS indigo desktop full (http://wiki.ros.org/indigo/Installation/Ubuntu).
2. Clone ode_simulator in your ROS worspace src folder.


## BASIC USAGE

Place a folder containing the info files for your system (see symToODE USAGE) in 'data/'.
From 'ode_simulator/' call

	$ ./dataToFun.sh name

where 'name' is the previous folder name.
On success 'src/name' will be created with an ode function for your system.
At the same time a target 'name_node' will be added in CMakeLists.txt.
This is the simulator node for your system.
Parameters:
• _u        The inputs topic (std_msgs::Float64MultiArray)

Published Topic:
• x         The state vector (std_msgs::Float64MultiArray)


## quad2dof TESTER

Calling

	$ roslaunch ode_simulator test_quad2dof.launch

will run the 'quad2dof_node' and 'test_quad2dof_node', which implements a hovering controller.
Also it lauches 'rqt_plot' presenting the quad's x,y,z coordinates.


## symToODE USAGE
	
	$ rosrun ode_simulator symToODE dataFolder outFolder

Where dataFolder is a folder containing the following files

• D.txt     The matrix D in the form:
                [d11, d12, ...
                 d21, d22, ...]

• W.txt     The vector W in the form:
                [w1
                 w2
                ...]

• P.txt     A file containing all the parameters used in D, W.
            eg.
                Ix = 0.0196
                Iy = 0.0196
                Iz = 0.0264
                m = 1.1
                ...

            NOTE: For QUAD + ARM  created with the given Maxima algorithm,
            you should give the folowing parameters:

                Quad: Ix, Iy, Iz, m, g, la, b, d
                Arm : mi, li, lcxi, lcyi, lczi, Ixxi, Iyyi, Izzi, Ixyi, Ixzi, Iyzi

            where i = {1, ..., n}, n the arm's dof.

• X.txt     The description of the state vector, as the states are used in D, W.
            eg.
                [x,vx, y,vy, z,vz, phi,wx, theta,wy, psi,wz, q1,qd1, q2,qd2]

• U.txt     The description of the inputs vector, as the inputs are used in D, W.
            eg.
                [ U1, U2, U3, U4, tau1, tau2 ]

On success the outFolder will contain 'odefun.h' and 'odefun.c'.

NOTE: All matrices may also be in Octave's or Maxima's format.
