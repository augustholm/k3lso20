# k3lso20
K3lso for the bachelors thesis EENX16-23-20 Robothund

This is the GitHub repository for the bachelor thesis project of group EENX16-23-20

Firstly follow Robin Fröjd's install proccess from his GitHub to get the simulation environment started and getting the correct version of all the packages that are needed to run this project.

Secondly get all the code from the "robot_gym" map and overwrite the code that exists in Robin's repository with our version. 

The PyBullet simulation is started by running the playground.py file in 
robot_gym/playground/playground.py.

From there you can choose the controller and environment for the simulation and also specify inputs for K3lso.

Some keybinds for pybullet:

w - seems to illustrate the robots different parts, and if you press "a" when in this mode you can see even more details.

s - Activates/deactivates shadows.

g - takes away the GUI to make it easier to see

p - Makes the terminal give the output(or similar): "Writing 10285 timings for thread 0
				  Writing 260 timings for thread 1
				  Writing 3356 timings for thread 2"

v - Does something, unsure of what

j - Shows a individual coordinatesystem of every model that makes up K3lso 

i - Lets you freely change the simulated view, if K3lsos cameras are on at the same time the simulation will be paused until the simulated view are back to being fixed onto K3lso. The camera will otherwise centered onto K3lsos body. 



K3lsos built-in computer has the password: robothund.