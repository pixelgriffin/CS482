# Installation instructions

You will need to have python, numpy, and scipy installed:

	sudo apt-get install python-numpy python-scipy python-opencv
	chmod +x sim.py

# Python running instructions

	./sim.py

# C++ compilation/running instructions

To make the simulator use the c++ code, change lines 14, 15 of sim.py to be:

	from kf_ros import kf_update
	from kf_ros import door_update

Every time you open a terminal cd to the KalmanFilter directory and type:

	. rosrc

To make (from KalmanFilter directory):

	make

To run (from KalmanFilter directory):

	roslaunch cpp_code.launch
	
	(in a second terminal)
	./sim.py


# Your Code

You should put all of your code in kf.py (python) or kalman.cpp (c++), any changes made to sim.py will be wiped away!
