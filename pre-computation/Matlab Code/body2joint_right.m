% Data sets used here are extraced from sdfastComputation plugin and EXCEL computations

% w_2_r.... is the rotation matrix from world reference frame to link reference frame (right links).
% Link reference frame is coincident with its center of gravity.
% r_....._l is BodytoJoint Vector represented in link reference frame (right links)

clc

%==========================================================================
% RIGHT LOWER BODY
%==========================================================================
% BodytoJoint Vector represented in world refrence frame.
% BodytoJoint = (Position of Upper Joint) - (Link CoG in world)

r_hip_1 = [-0.000124	-0.0782	0] ;
r_hip_2 = [0	0	0.03045 ];
r_hip_3 = [0	0	0];
r_upper_leg = [-0.00144	0.000393	0.15943];
r_lower_leg = [-0.001818	0.002113	0.1071] ;
r_ankle_1 = [-0.017202	-0.001603	0.0054 ];
r_ankle_2 = [0	0	0.037];
r_foot = [-0.024067	0.000652	-0.028415];


%==========================================================================
% Rotation matrix from world to link reference frame.


w_2_root =[0.999999 0.00159265 0 
-0.00159265 -0.999999 0 
0 0 1 ] ;

w_2_r_hip_1 = [6.32679e-06 -1.00596e-08 -1 
-0.00158735 -0.999996 1.67887e-11 
-0.999999 0.00158735 -6.3268e-06 ] ;

w_2_r_hip_2 = [0.999999 -0.00159 7e-06 
-0.00159 -0.999999 -2.65359e-06 
7.00421e-06 2.64246e-06 -1] ;

w_2_r_hip_3 = [0.999999 -0.00159 7e-06 
-0.00159 -0.999999 -2.65359e-06 
7.00421e-06 2.64246e-06 -1] ;

w_2_r_upper_leg = [0.999999 -0.00159531 7e-06 
-6.9899e-06 -6.73204e-07 1 
-0.00159531 -0.999999 6.32679e-06] ;

w_2_r_lower_leg = [-1.54807e-05 5.18765e-06 1 
0.999999 1.19936 1.5489e-05 
0.00159998 0.999999 -5.16288e-06] ;

w_2_r_ankle_1 = [-1.54807e-05 5.18765e-06 1 
0.001591 0.599678 -5.16302e-06 
-0.999999 0.001591 -1.54889e-05] ;

w_2_r_ankle_2 = [0.999999 -0.00159 1.4e-05 
-0.00159 -0.999999 2.65359e-06 
1.39958e-05 -2.67585e-06 -1] ;

w_2_r_foot = [0.999999 -0.00159 1.4e-05 
-0.00159 -0.999999 2.65359e-06 
1.39958e-05 -2.67585e-06 -1] ;

%==========================================================================
% BodytoJoint Vector represented in link refrence frame.

r_hip_1_l = w_2_r_hip_1 * r_hip_1' ;
r_hip_2_l = w_2_r_hip_2 * r_hip_2' ;
r_hip_3_l = w_2_r_hip_3 * r_hip_3' ;
r_upper_leg_l = w_2_r_upper_leg * r_upper_leg' ;
r_lower_leg_l = w_2_r_lower_leg * r_lower_leg' ;
r_ankle_1_l = w_2_r_ankle_1 * r_ankle_1' ;
r_ankle_2_l = w_2_r_ankle_2 * r_ankle_2' ;
r_foot_l = w_2_r_foot * r_foot' ;


fprintf('Right Lower Body: Body to Joint Vectors\n\n')
fprintf('r_hip_1_l = %f\t%f\t%f\n\n',r_hip_1_l);
fprintf('r_hip_2_l = %f\t%f\t%f\n\n',r_hip_2_l);
fprintf('r_hip_3_l = %f\t%f\t%f\n\n',r_hip_3_l);
fprintf('r_upper_leg_l = %f\t%f\t%f\n\n',r_upper_leg_l);
fprintf('r_lower_leg_l = %f\t%f\t%f\n\n',r_lower_leg_l);
fprintf('r_ankle_1_l = %f\t%f\t%f\n\n',r_ankle_1_l);
fprintf('r_ankle_2_l = %f\t%f\t%f\n\n',r_ankle_2_l);
fprintf('r_foot_l = %f\t%f\t%f\n\n',r_foot_l);



%==========================================================================
% LEFT UPPER BODY
%==========================================================================
% BodytoJoint Vector represented in world refrence frame.

r_shoulder_1 = [0.005811	-0.017804	0.000587];
r_shoulder_2 = [-0.003783	0.007444	0.015658];
r_shoulder_3 = [-0.031365	0.018311	0.033926];
r_upper_arm = [-0.016125	0.011108	0.022575];
r_elbow_1 = [0.000383	-0.003411	0.002184];
r_forearm = [-0.062064	0.019573	0.011991];
r_wrist_1 = [0	0	0];
r_hand_base_link = [-0.043737	0.007326	0.029004];

%==========================================================================
% Rotation matrix from world to link reference frame.

w_2_r_shoulder_1 = [-0.480159 -0.127837 0.867816 
-0.257283 0.966331 -3.94732e-06 
-0.838602 -0.223276 -0.496885 ] ;

w_2_r_shoulder_2 = [-0.015312 -0.902127 0.431198 
0.838602 -0.0101353 0.496888 
-0.544529 0.369212 0.753107 ];

w_2_r_shoulder_3 = [0.806066 -0.0178191 0.591557 
-0.231833 -1.14503 0.28791 
0.544528 -0.369217 -0.753105];

w_2_r_upper_arm = [ 0.806066 -0.0178191 0.591557 
-0.231833 -1.14503 0.28791 
0.544528 -0.369217 -0.753105 ];

w_2_r_elbow_1 = [-0.185315 -0.248364 -0.950775 
-0.231829 -0.749445 0.287907 
-0.954942 0.27377 0.114612 ];

w_2_r_forearm = [0.23183 0.929176 -0.2879 
-0.954942 0.248681 0.114613 
0.185314 0.248357 0.950777];

w_2_r_wrist_1 = [-0.954942 0.273769 0.114617 
0.185319 0.384045 0.950774 
0.231826 0.929175 -0.287908 ] ;

w_2_r_hand_base_link = [0.231377 0.929086 -0.288554 
-0.787604 0.242085 0.505046 
0.57109 0.11041 0.813428 ] ;

%==========================================================================
% BodytoJoint Vector represented in link refrence frame.

r_shoulder_1_l = w_2_r_shoulder_1 * r_shoulder_1' ;
r_shoulder_2_l = w_2_r_shoulder_2 * r_shoulder_2' ;
r_shoulder_3_l = w_2_r_shoulder_3 * r_shoulder_3' ;
r_upper_arm_l = w_2_r_upper_arm * r_upper_arm' ;
r_elbow_1_l = w_2_r_elbow_1 * r_elbow_1' ;
r_forearm_l = w_2_r_forearm * r_forearm' ;
r_wrist_1_l = w_2_r_wrist_1 * r_wrist_1' ;
r_hand_base_link_l = w_2_r_hand_base_link * r_hand_base_link' ;

fprintf('Right Upper Body: Body to Joint Vectors\n\n')

fprintf('r_shoulder_1_l = %f\t%f\t%f\n\n',r_shoulder_1_l);
fprintf('r_shoulder_2_l = %f\t%f\t%f\n\n',r_shoulder_2_l);
fprintf('r_shoulder_3_l = %f\t%f\t%f\n\n',r_shoulder_3_l);
fprintf('r_upper_arm_l = %f\t%f\t%f\n\n',r_upper_arm_l);
fprintf('r_elbow_1_l = %f\t%f\t%f\n\n',r_elbow_1_l);
fprintf('r_forearm_l = %f\t%f\t%f\n\n',r_forearm_l);
fprintf('r_wrist_1_l = %f\t%f\t%f\n\n',r_wrist_1_l);
fprintf('r_hand_base_link_l = %f\t%f\t%f\n\n',r_hand_base_link_l);
