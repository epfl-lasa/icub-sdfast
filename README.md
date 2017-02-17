# Description 

This package provides an interface to extract required information for general equations of motion of the whole robot (like mass matrix) from the SDFAST output codes.

# Prerequisite
The model of icub as a floating base robot should be run seperately in SD/FAST. Then, the output files of sdfast including c codes should be provided in `icub-sdfast/sdfast/src/`. In this directory, sdfast_model.sd is the description file which is uasd as the input file for SD/FAST to generate the symbolic dynamic.
sdfast_model_d.c, sdfast_model_s.c, sdfast_model_i are the outputs of SD/FAST which are needed here to compute mass matrix, gravitational force and ....

# Test the functionality
navigate to icub-sdfast folder, then:

	mkdir build
	cd build
	cmake ..
	make 
	./test

# Code Implementation
In icub-sdfast/codes/src/Model.cpp, "get_massmat" and "get_frcmat" return the mass matrix and gravitational, corriolis and ... forces respectively.
The Model class(model.cpp and model.hpp) interfaces with the SDFAST library to use its functions.

Before using this interface, following steps should be done:

1 - The number of states (AIR_N_Q) and number of state derivatives (AIR_N_U), also the number of bodies and joints should be defined in icub-sdfast/sdfast/include/Description.hpp

2-  Description of state vector and its derivative, as well as body and joint indices can be defined in Description.hpp. This step is optional.

3-  Minimum and maximum feasible joint angles should be specified in Model.cpp (Model::init).






