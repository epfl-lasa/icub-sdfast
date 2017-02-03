% Data sets used here are extraced from sdfastComputation plugin and EXCEL computations.

% w_2_l.... is the rotation matrix from world reference frame to link reference frame (left links).
% Link reference frame is coincident with its center of gravity.
% l_....._l is inboard2joint Vector represented in the parent link reference frame
% inboard2joint vector for each link should be defined in its parent link reference frame. 

clc

%==========================================================================
% LEFT LOWER BODY
%==========================================================================
% InboardtoJoint Vector represented in world refrence frame.
% InboardtoJoint = (Position of Upper Joint) - (Parent Link CoG in world)

l_hip_1 = [0.000108	0.0681	-0.1199] ;
l_hip_2 = [0.000125	0.0782	0 ];
l_hip_3 = [-0.000001	0	-0.03605];

l_upper_leg = [0	0	0.0665];
l_lower_leg = [-0.00144	-0.000388	-0.06417] ;
l_ankle_1 = [-0.001823	-0.002108	-0.1059];
l_ankle_2 = [-0.149288	0.001658	0.0054];
l_foot = [-0.000001	0	0];

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
% Inboard2Joint Vector represented in parent link refrence frame.

l_hip_1_l = w_2_root * l_hip_1' ;
l_hip_2_l = w_2_l_hip_1 * l_hip_2' ;
l_hip_3_l = w_2_l_hip_2 * l_hip_3' ;
l_upper_leg_l = w_2_l_hip_3 * l_upper_leg' ;
l_lower_leg_l = w_2_l_upper_leg * l_lower_leg' ;
l_ankle_1_l = w_2_l_lower_leg * l_ankle_1' ;
l_ankle_2_l = w_2_l_ankle_1 * l_ankle_2' ;
l_foot_l = w_2_l_ankle_2 * l_foot' ;


fprintf('Left Lower Body: Inboard to Joint Vectors\n\n')
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
% Inboard2Joint Vector represented in world refrence frame.

l_shoulder_1 = [0.003197	0.110256	0.0683];
l_shoulder_2 = [0.005864	0.017787	0.000596];
l_shoulder_3 = [-0.003803	-0.007601	0.015577];
l_upper_arm = [0.017314	0.0068	-0.007862];

l_elbow_1 = [0.029879	0.01993	-0.040965];
l_forearm = [-0.002373	0.007145	-0.012077];
l_wrist_1 = [0.069107	0.017795	-0.003747];
l_hand_base_link = [0 0 0];


%=========================================================================
% Rotation matrix from world to link reference frame.

w_2_chest = [0.999999 -0.00159 7e-06 
-7.00583e-06 -1.06732e-05 1 
-0.00159 -0.999999 -3.67321e-06];

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
% Inboard2Joint Vector represented in parent link refrence frame.

l_shoulder_1_l = w_2_chest * l_shoulder_1' ;
l_shoulder_2_l = w_2_l_shoulder_1 * l_shoulder_2' ;
l_shoulder_3_l = w_2_l_shoulder_2 * l_shoulder_3' ;
l_upper_arm_l = w_2_l_shoulder_3 * l_upper_arm' ;
l_elbow_1_l = w_2_l_upper_arm * l_elbow_1' ;
l_forearm_l = w_2_l_elbow_1 * l_forearm' ;
l_wrist_1_l = w_2_l_forearm * l_wrist_1' ;
l_hand_base_link_l = w_2_l_wrist_1 * l_hand_base_link' ;


fprintf('Right Upper Body: Inboard to Joint Vectors\n\n')

fprintf('l_shoulder_1_l = %f\t%f\t%f\n\n',l_shoulder_1_l);
fprintf('l_shoulder_2_l = %f\t%f\t%f\n\n',l_shoulder_2_l);
fprintf('l_shoulder_3_l = %f\t%f\t%f\n\n',l_shoulder_3_l);
fprintf('l_upper_arm_l = %f\t%f\t%f\n\n',l_upper_arm_l);
fprintf('l_elbow_1_l = %f\t%f\t%f\n\n',l_elbow_1_l);
fprintf('l_forearm_l = %f\t%f\t%f\n\n',l_forearm_l);
fprintf('l_wrist_1_l = %f\t%f\t%f\n\n',l_wrist_1_l);
fprintf('l_hand_base_link_l = %f\t%f\t%f\n\n',l_hand_base_link_l);