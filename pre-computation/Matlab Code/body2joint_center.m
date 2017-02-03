% Data sets used here are extraced from sdfastComputation plugin and EXCEL computations.

% w_2_.... is the rotation matrix from world reference frame to link reference frame (central part).
% Link reference frame is coincident with its center of gravity.
% ....._l is BodytoJoint Vector represented in link reference frame

clc

%==========================================================================
% CENTRAL PART (TORSO+CHEST+NECK+HEAD)
%==========================================================================
% BodytoJoint Vector represented in world refrence frame.
% BodytoJoint = (Position of Upper Joint) - (Link CoG in world)


torso_1 = [0	0	0.032];
torso_2 = [0    0   0   ];
chest = [-0.002228	0.000004	-0.075];
neck_1 = [0   0   0];
neck_2 = [0   0   0];
head = [0.018502	-0.000028	-0.1108];

%==========================================================================
% Rotation Matrix from world to link reference frame 

w_2_torso_1 = [-6.32676e-06 1.00953e-08 1 
0.00159831 1 1.67886e-11 
-0.999999 0.00159831 -6.32677e-06];

w_2_torso_2 = [-0.00159898 -0.999999 -4e-06 
-0.999999 0.00159898 -2.65359e-06 
2.65998e-06 3.99575e-06 -1];

w_2_chest = [0.999999 -0.00159 7e-06 
-7.00583e-06 -1.06732e-05 1 
-0.00159 -0.999999 -3.67321e-06];

w_2_neck_1 = [-6.32676e-06 1.00827e-08 1 
-0.00158765 -0.999993 3.79606e-11 
0.999999 -0.00158765 6.32677e-06 ];

w_2_neck_2 = [0.00158633 0.999999 -4e-06 
-0.999999 0.00158633 -1.5e-05 
-1.49936e-05 4.02379e-06 1];

w_2_head = [-0.999999 0.00158465 -1.5e-05 
1.50058e-05 1.86732e-05 -1 
-0.00158465 -0.999999 -3.67321e-06] ;

%==========================================================================
% BodytoJoint Vector represented in link refrence frame.

torso_1_l = w_2_torso_1 * torso_1';
torso_2_l = w_2_torso_2 * torso_2';
chest_l = w_2_chest * chest';
neck_1_l = w_2_neck_1 * neck_1';
neck_2_l = w_2_neck_2 * neck_2';
head_l = w_2_head * head';


fprintf('Central Lower Body: Body to Joint Vectors\n\n')
fprintf('torso_1_l = %f\t%f\t%f\n\n',torso_1_l);
fprintf('torso_2_l = %f\t%f\t%f\n\n',torso_2_l);
fprintf('chest_l = %f\t%f\t%f\n\n',chest_l);
fprintf('neck_1_l = %f\t%f\t%f\n\n',neck_1_l);
fprintf('neck_2_l = %f\t%f\t%f\n\n',neck_2_l);
fprintf('head_l = %f\t%f\t%f\n\n',head_l);

