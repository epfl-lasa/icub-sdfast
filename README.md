# Description 

This package provides an interface to extract required information for general equations of motion of the whole robot (like the mass matrix) from the SDFAST output codes.

# Prerequisite
The model of iCub as a floating base robot should be run separately in SD/FAST. Then, the output files of sdfast including c codes should be provided in `icub-sdfast/sdfast/src/`. In this directory, [sdfast_model.sd](https://github.com/epfl-lasa/icub-sdfast/blob/master/sdfast/src/sdfast_model.sd) is the description file which is used as the input file for SD/FAST to generate the dynamic of the robot. [sdfast_model_d.c]( https://github.com/epfl-lasa/icub-sdfast/blob/master/sdfast/src/sdfast_model_d.c), [sdfast_model_s.c](https://github.com/epfl-lasa/icub-sdfast/blob/master/sdfast/src/sdfast_model_s.c), [sdfast_model_i](https://github.com/epfl-lasa/icub-sdfast/blob/master/sdfast/src/sdfast_model_i) are the outputs of SD/FAST which are needed here to compute mass matrix, gravitational force and ....

# Test the functionality
navigate to icub-sdfast folder, then:

	mkdir build
	cd build
	cmake ..
	make 
	./test
	
Before using any SD/FAST functtions, initial states of the system including initial position and velocity should be defined using the following command:

	iCubModel.set_state(0, ref_pos, ref_vel)
	

# Code Implementation
In `icub-sdfast/sdfast/src/Model.cpp`, `get_massmat` and `get_frcmat` return the mass matrix and gravitational, Coriolis and ... forces respectively.
The Model class ([model.cpp](https://github.com/epfl-lasa/icub-sdfast/blob/master/codes/src/Model.cpp) and [model.hpp](https://github.com/epfl-lasa/icub-sdfast/blob/master/codes/include/Model.hpp)) interfaces with the SDFAST library to use its functions.

Before using this interface, the following steps should be done:

1. The number of states (`AIR_N_Q`) and the number of state derivatives (`AIR_N_U`) and the number of bodies and joints should be defined in [Description.hpp](https://github.com/epfl-lasa/icub-sdfast/blob/master/sdfast/include/Description.hpp).

2.  The description of the state vector and its derivative, as well as body and joint indices,  can be defined in [Description.hpp](https://github.com/epfl-lasa/icub-sdfast/blob/master/sdfast/include/Description.hpp). This step is optional.

3.  Minimum and maximum feasible joint angles should be specified in [Model.cpp](https://github.com/epfl-lasa/icub-sdfast/blob/master/codes/src/Model.cpp) (`Model::init`).
