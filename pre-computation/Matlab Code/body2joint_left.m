% Data sets used here are extraced from sdfastComputation plugin and EXCEL computations.

% w_2_l.... is the rotation matrix from world reference frame to link reference frame (left links).
% Link reference frame is coincident with its center of gravity.
% l_....._l is BodytoJoint Vector represented in link reference frame

clc

%==========================================================================
% LEFT LOWER BODY
%==========================================================================
% BodytoJoint Vector represented in world refrence frame.
% BodytoJoint = (Position of Upper Joint) - (Link CoG in world)

l_hip_1 = [0.000125	0.0782	0] ;
l_hip_2 = [-0.000001	0	0.03045 ];
l_hip_3 = [0	0	0];
l_upper_leg = [-0.001442	-0.000387	0.15943];
l_lower_leg = [-0.001825	-0.002106	0.1071] ;
l_ankle_1 = [ -0.149288	0	0.037];
l_ankle_2 = [-0.000001	0	0.037];
l_foot = [-0.02407	-0.000575	-0.028415];

%==========================================================================
% Rotation matrix from world to link reference frame.

w_2_root =[0.999999 0.00159265 0 
-0.00159265 -0.999999 0 
0 0 1 ] ;

w_2_l_hip_1 = [6.32676e-06 -1.00931e-08 -1 
0.00159831 1 1.89803e-11 
0.999999 -0.00159831 6.32677e-06 ];

w_2_l_hip_2 = [0.999999 -0.00159531 7e-06 
-0.00159531 -0.999999 2.65359e-06 
6.99576e-06 -2.66475e-06 -1 ] ;

w_2_l_hip_3 = [0.999999 -0.00159531 7e-06 
-0.00159531 -0.999999 2.65359e-06 
6.99576e-06 -2.66475e-06 -1 ];

w_2_l_upper_leg = [ 0.999999 -0.00159531 7e-06 
-6.9899e-06 -6.73204e-07 1 
-0.00159531 -0.999999 6.32679e-06] ;

w_2_l_lower_leg = [-1.41461e-05 8.15184e-06 1 
0.999999 1.1828 1.41591e-05 
0.00159798 0.999999 -8.12922e-06 ];

w_2_l_ankle_1 = [-1.41461e-05 8.15184e-06 1 
-0.00160165 -0.319999 8.12917e-06 
0.999999 -0.00160165 1.41591e-05] ;

w_2_l_ankle_2 = [0.999999 -0.00160531 7e-06 
-0.00160531 -0.999999 2.65359e-06 
6.99573e-06 -2.66482e-06 -1] ;

w_2_l_foot = [0.999999 -0.00160531 7e-06 
-0.00160531 -0.999999 2.65359e-06 
6.99573e-06 -2.66482e-06 -1] ;

%=====================================================================
% BodytoJoint Vector represented in link refrence frame.

l_hip_1_l = w_2_l_hip_1 * l_hip_1';
l_hip_2_l = w_2_l_hip_2 * l_hip_2' ;
l_hip_3_l = w_2_l_hip_3 * l_hip_3' ;
l_upper_leg_l = w_2_l_upper_leg * l_upper_leg' ;
l_lower_leg_l = w_2_l_lower_leg * l_lower_leg' ;
l_ankle_1_l = w_2_l_ankle_1 * l_ankle_1' ;
l_ankle_2_l = w_2_l_ankle_2 * l_ankle_2' ;
l_foot_l = w_2_l_foot * l_foot' ;

fprintf('Left Lower Body: Body to Joint Vectors\n\n')
fprintf('l_hip_1_l = %f\t%f\t%f\n\n',l_hip_1_l);
fprintf('l_hip_2_l = %f\t%f\t%f\n\n',l_hip_2_l);
fprintf('l_hip_3_l = %f\t%f\t%f\n\n',l_hip_3_l);
fprintf('l_upper_leg_l = %f\t%f\t%f\n\n',l_upper_leg_l);
fprintf('l_lower_leg_l = %f\t%f\t%f\n\n',l_lower_leg_l);
fprintf('l_ankle_1_l = %f\t%f\t%f\n\n',l_ankle_1_l);
fprintf('l_ankle_2_l = %f\t%f\t%f\n\n',l_ankle_2_l);
fprintf('l_foot_l = %f\t%f\t%f\n\n',l_foot_l);

%==========================================================================
% LEFT UPPER BODY
%===================================================================
% BodytoJoint Vector represented in world refrence frame.

l_shoulder_1 = [0.005864	0.017787	0.000596];
l_shoulder_2 = [-0.003803	-0.007601	0.015577];
l_shoulder_3 = [-0.031885	-0.018417	0.034476];
l_upper_arm = [-0.016113	-0.011041	0.022507];
l_elbow_1 = [0.000395	0.00341	0.002184];
l_forearm = [-0.062125	-0.019376	0.011991];
l_wrist_1 = [0	0	0];
l_hand_base_link = [-0.043457	-0.007082	0.028905];


%=========================================================================
% Rotation matrix from world to link reference frame.


w_2_l_shoulder_1 = [-0.47975 0.129364 0.867816 
-0.260354 -0.965517 -1.72974e-06 
0.837888 -0.22594 0.496885] ;

w_2_l_shoulder_2 = [-0.0124385 0.902171 0.431199 
0.83789 0.0147808 0.496884 
0.545698 0.367478 -0.75311 ];

w_2_l_shoulder_3 = [0.806118 0.0152561 0.591558 
0.228873 -0.722706 -0.287905 
0.545702 0.367477 -0.753107];

w_2_l_upper_arm = [ 0.806118 0.0152561 0.591558 
0.228873 -0.722706 -0.287905 
0.545702 0.367477 -0.753107 ];

w_2_l_elbow_1 = [0.184523 -0.248949 0.950776 
0.228873 0.30577 -0.287904 
0.955808 0.270731 -0.114611 ];

w_2_l_forearm = [-0.228873 0.929909 0.2879 
0.955807 0.229039 -0.114615 
-0.184525 0.248945 -0.950777 ];

w_2_l_wrist_1 = [0.955808 0.27073 -0.114616 
-0.184527 0.1733 -0.950775 
-0.228869 0.929908 0.287905 ] ;

w_2_l_hand_base_link = [0.228418 -0.929817 -0.288556 
-0.787814 -0.166072 0.50634 
-0.571992 0.111671 -0.812622 ] ;

%==================================================================
% BodytoJoint Vector represented in link refrence frame.

l_shoulder_1_l = w_2_l_shoulder_1 * l_shoulder_1' ;
l_shoulder_2_l = w_2_l_shoulder_2 * l_shoulder_2' ;
l_shoulder_3_l = w_2_l_shoulder_3 * l_shoulder_3' ;
l_upper_arm_l = w_2_l_upper_arm * l_upper_arm' ;
l_elbow_1_l = w_2_l_elbow_1 * l_elbow_1' ;
l_forearm_l = w_2_l_forearm * l_forearm' ;
l_wrist_1_l = w_2_l_wrist_1 * l_wrist_1' ;
l_hand_base_link_l = w_2_l_hand_base_link * l_hand_base_link' ;


fprintf('Left Upper Body: Body to Joint Vectors\n\n')

fprintf('l_shoulder_1_l = %f\t%f\t%f\n\n',l_shoulder_1_l);
fprintf('l_shoulder_2_l = %f\t%f\t%f\n\n',l_shoulder_2_l);
fprintf('l_shoulder_3_l = %f\t%f\t%f\n\n',l_shoulder_3_l);
fprintf('l_upper_arm_l = %f\t%f\t%f\n\n',l_upper_arm_l);
fprintf('l_elbow_1_l = %f\t%f\t%f\n\n',l_elbow_1_l);
fprintf('l_forearm_l = %f\t%f\t%f\n\n',l_forearm_l);
fprintf('l_wrist_1_l = %f\t%f\t%f\n\n',l_wrist_1_l);
fprintf('l_hand_base_link_l = %f\t%f\t%f\n\n',l_hand_base_link_l);