% Data sets used here are extraced from sdfastComputation plugin and EXCEL computations.

% w_2_r.... is the rotation matrix from world reference frame to link reference frame (right links).
% Link reference frame is coincident with its center of gravity.
% r_....._l is inboard2joint Vector represented in the parent link reference frame
% inboard2joint vector for each link should be defined in its parent link reference frame. 

clc

%==========================================================================
% RIGHT LOWER BODY
%==========================================================================
% InboardtoJoint Vector represented in world refrence frame.
% InboardtoJoint = (Position of Upper Joint) - (Parent Link CoG in world)

r_hip_1 = [-0.000108	-0.0681	-0.1199] ;
r_hip_2 = [-0.000124	-0.0782	0];
r_hip_3 = [0	0	-0.03605];

r_upper_leg = [0	0	0.0665];
r_lower_leg = [-0.001438	0.000392	-0.06417] ;
r_ankle_1 = [-0.001816	0.002112	-0.1059];
r_ankle_2 = [-0.017202	-0.001603	0.0054];
r_foot = [0.000001	0	0];

%==========================================================================
% Rotation matrix from world to link reference frame.

w_2_root =[0.999999 0.00159265 0 
-0.00159265 -0.999999 0 
0 0 1 ] ;

w_2_r_hip_1 = [6.32676e-06 -1.00931e-08 -1 
0.00159831 1 1.89803e-11 
0.999999 -0.00159831 6.32677e-06 ];

w_2_r_hip_2 = [0.999999 -0.00159531 7e-06 
-0.00159531 -0.999999 2.65359e-06 
6.99576e-06 -2.66475e-06 -1 ] ;

w_2_r_hip_3 = [0.999999 -0.00159531 7e-06 
-0.00159531 -0.999999 2.65359e-06 
6.99576e-06 -2.66475e-06 -1 ];

w_2_r_upper_leg = [ 0.999999 -0.00159531 7e-06 
-6.9899e-06 -6.73204e-07 1 
-0.00159531 -0.999999 6.32679e-06] ;

w_2_r_lower_leg = [-1.41461e-05 8.15184e-06 1 
0.999999 1.1828 1.41591e-05 
0.00159798 0.999999 -8.12922e-06 ];

w_2_r_ankle_1 = [-1.41461e-05 8.15184e-06 1 
-0.00160165 -0.319999 8.12917e-06 
0.999999 -0.00160165 1.41591e-05] ;

w_2_r_ankle_2 = [0.999999 -0.00160531 7e-06 
-0.00160531 -0.999999 2.65359e-06 
6.99573e-06 -2.66482e-06 -1] ;

w_2_r_foot = [0.999999 -0.00160531 7e-06 
-0.00160531 -0.999999 2.65359e-06 
6.99573e-06 -2.66482e-06 -1] ;

%=====================================================================
% Inboard2Joint Vector represented in parent link refrence frame.

r_hip_1_l = w_2_root * r_hip_1' ;
r_hip_2_l = w_2_r_hip_1 * r_hip_2' ;
r_hip_3_l = w_2_r_hip_2 * r_hip_3' ;
r_upper_leg_l = w_2_r_hip_3 * r_upper_leg' ;
r_lower_leg_l = w_2_r_upper_leg * r_lower_leg' ;
r_ankle_1_l = w_2_r_lower_leg * r_ankle_1' ;
r_ankle_2_l = w_2_r_ankle_1 * r_ankle_2' ;
r_foot_l = w_2_r_ankle_2 * r_foot' ;

fprintf('Right Lower Body: Inboard to Joint Vectors\n\n')
fprintf('r_hip_1_l = %f\t%f\t%f\n\n',r_hip_1_l);
fprintf('r_hip_2_l = %f\t%f\t%f\n\n',r_hip_2_l);
fprintf('r_hip_3_l = %f\t%f\t%f\n\n',r_hip_3_l);
fprintf('r_upper_leg_l = %f\t%f\t%f\n\n',r_upper_leg_l);
fprintf('r_lower_leg_l = %f\t%f\t%f\n\n',r_lower_leg_l);
fprintf('r_ankle_1_l = %f\t%f\t%f\n\n',r_ankle_1_l);
fprintf('r_ankle_2_l = %f\t%f\t%f\n\n',r_ankle_2_l);
fprintf('r_foot_l = %f\t%f\t%f\n\n',r_foot_l);


%==========================================================================
% RIGHT UPPER BODY
%===================================================================
% Inboard2Joint Vector represented in world refrence frame.

r_shoulder_1 = [0.002845	-0.110264	0.0683];
r_shoulder_2 = [0.005811	-0.017804	0.000587];
r_shoulder_3 = [-0.003783	0.007444	0.015658];
r_upper_arm = [0.017754	-0.007063	-0.008412];
r_elbow_1 = [0.029768	-0.020009	-0.040897];
r_forearm = [-0.002397	-0.007137	-0.012077];
r_wrist_1 = [0.06905	-0.018016	-0.003746];
r_hand_base_link = [0 0 0];


%=========================================================================
% Rotation matrix from world to link reference frame.

w_2_chest = [0.999999 -0.00159 7e-06 
-7.00583e-06 -1.06732e-05 1 
-0.00159 -0.999999 -3.67321e-06];

w_2_r_shoulder_1 = [-0.47975 0.129364 0.867816 
-0.260354 -0.965517 -1.72974e-06 
0.837888 -0.22594 0.496885] ;

w_2_r_shoulder_2 = [-0.0124385 0.902171 0.431199 
0.83789 0.0147808 0.496884 
0.545698 0.367478 -0.75311 ];

w_2_r_shoulder_3 = [0.806118 0.0152561 0.591558 
0.228873 -0.722706 -0.287905 
0.545702 0.367477 -0.753107];

w_2_r_upper_arm = [ 0.806118 0.0152561 0.591558 
0.228873 -0.722706 -0.287905 
0.545702 0.367477 -0.753107 ];

w_2_r_elbow_1 = [0.184523 -0.248949 0.950776 
0.228873 0.30577 -0.287904 
0.955808 0.270731 -0.114611 ];

w_2_r_forearm = [-0.228873 0.929909 0.2879 
0.955807 0.229039 -0.114615 
-0.184525 0.248945 -0.950777 ];

w_2_r_wrist_1 = [0.955808 0.27073 -0.114616 
-0.184527 0.1733 -0.950775 
-0.228869 0.929908 0.287905 ] ;

w_2_r_hand_base_link = [0.228418 -0.929817 -0.288556 
-0.787814 -0.166072 0.50634 
-0.571992 0.111671 -0.812622 ] ;

%==================================================================
% Inboard2Joint Vector represented in parent link refrence frame.

r_shoulder_1_l = w_2_chest * r_shoulder_1' ;
r_shoulder_2_l = w_2_r_shoulder_1 * r_shoulder_2' ;
r_shoulder_3_l = w_2_r_shoulder_2 * r_shoulder_3' ;
r_upper_arm_l = w_2_r_shoulder_3 * r_upper_arm' ;
r_elbow_1_l = w_2_r_upper_arm * r_elbow_1' ;
r_forearm_l = w_2_r_elbow_1 * r_forearm' ;
r_wrist_1_l = w_2_r_forearm * r_wrist_1' ;
r_hand_base_link_l = w_2_r_wrist_1 * r_hand_base_link' ;

fprintf('Right Upper Body: Inboard to Joint Vectors\n\n')

fprintf('r_shoulder_1_l = %f\t%f\t%f\n\n',r_shoulder_1_l);
fprintf('r_shoulder_2_l = %f\t%f\t%f\n\n',r_shoulder_2_l);
fprintf('r_shoulder_3_l = %f\t%f\t%f\n\n',r_shoulder_3_l);
fprintf('r_upper_arm_l = %f\t%f\t%f\n\n',r_upper_arm_l);
fprintf('r_elbow_1_l = %f\t%f\t%f\n\n',r_elbow_1_l);
fprintf('r_forearm_l = %f\t%f\t%f\n\n',r_forearm_l);
fprintf('r_wrist_1_l = %f\t%f\t%f\n\n',r_wrist_1_l);
fprintf('r_hand_base_link_l = %f\t%f\t%f\n\n',r_hand_base_link_l);


