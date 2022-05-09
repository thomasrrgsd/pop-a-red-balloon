# Pop A Red Balloon
Note: The official demonstation that was performed used commit: aeaefc499b47a17f44b13faf8e44ce02b4479059.
This commit can be found in the NCSU github repository, but is only accessible to NCSU accounts.
Later commits are for organizing source files and adding comments, and the final commit in the NCSU repository
is the first commit in this repository.

## ./main.py
This file is where the code begins and an example of use is "python main.py --connect :14551".
This code uses dronekit which requires the use of python2. Doesn't work with python3.5 or newer.
Did not test between 2.7 and 3.5.

## ./navigation.py
Called from main.py and is responsible for the drones movements in the grid sweep, slow approach, and balloon pop verification.
Receives information about red object from videoproc.py.

## ./videoproc.py
Opens video feed and looks for red object. Sends all red object information to navigation.py. Called from navigation.py but is closed
by the user manually when the mission is over using the 'q' key.

## ./func/func_navi.py
Contains all function definitions associated with dronekit and drone operation.

## ./func/generate_grid_waypoints.py
Based on initial longitude and latitude input, yaw direction, and sweep row depth, generates a grid facing the yaw direction, with the number of rows equal to sweep depth, and the top right corner waypoint being the initial latitude and longitude.

## ./old/*
This directory is filled with code that was used to test different parts or features, such as the sockets. This code is unimportant and is only left as something to glance at if the user wants to. All features that are implemented in the final code are explained in the actual files.
