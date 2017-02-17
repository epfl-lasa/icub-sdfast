#include "Model.hpp"
using namespace std;

Cmatrix quat2dc(Cvector st)
{
    double dc_cp[3][3];
    sdfast_model_quat2dc(st[0],st[1],st[2],st[3],dc_cp);
    return matrix3_convert(dc_cp);
}

Cvector3 dc2ang(Cmatrix dc)
{
    Cvector3 stang;
    double dc_cp[3][3];
    matrix3_convert(dc,dc_cp);
    sdfast_model_dc2ang(dc_cp,&stang[0],&stang[1],&stang[2]);
    return stang;
}

Cmatrix ang2dc(Cvector3 stang)
{
    double dc_cp[3][3];
    sdfast_model_ang2dc(stang[0],stang[1],stang[2],dc_cp);
    return matrix3_convert(dc_cp);
}

Cvector3 quat2ang(Cvector st)
{
    Cmatrix DC = quat2dc(st);
    return dc2ang(DC);
}

Cvector dc2quat(Cmatrix dc)
{
    Cvector st(4);
    double dc_cp[3][3];
    matrix3_convert(dc,dc_cp);
    sdfast_model_dc2quat(dc_cp,&st[0],&st[1],&st[2],&st[3]);
    return st;
}

Cvector ang2quat(Cvector3 stang)
{
    Cmatrix DC = ang2dc(stang);
    return dc2quat(DC);
}


Cmatrix skew_symmetric(Cvector3 in)
{
    Cmatrix res(3,3);
    res.setZero();
    res(0,1) = -in[2];
    res(1,0) = in[2];
    res(0,2) = in[1];
    res(2,0) = -in[1];
    res(1,2) = -in[0];
    res(2,1) = in[0];
    return res;
}

Cvector3 skew_symmetric_inv(Cmatrix &in)
{
    Cvector3 res;
    res[0] = in(2,1);
    res[1] = in(0,2);
    res[2] = in(1,0);
    return res;
}

void Model::init()
{
	sdfast_model_init();
	qmin = Cvector(AIR_N_U);
	qmax = Cvector(AIR_N_U);

	qmax << inf, inf, inf, inf, inf, inf,

			1.46608, 0.680678, 1.02974,                     // torso pitch, roll, yaw
			0.0872665, 2.80649, 1.74533,		        // left shoulder pitch ,roll, yaw
			1.85005, 				        //left elbow
			0.872665, 0.174533, 0.436332 ,                  //left wrist prosup, pitch, yaw
			2.30383, 2.07694, 1.37881, 		        //left hip pitch, roll, yaw
			0.401426, 0.366519,	0.418879,		//left knee, ankle pitch, ankle roll
			0.0872665, 2.80649, 1.74533,			// right shoulder pitch ,roll, yaw
			1.85005, 					//right elbow
			0.872665, 0.174533, 0.436332,			//right wrist prosup, pitch, yaw
			2.30383, 2.07694, 1.37881, 			//right hip pitch, roll, yaw
			0.401426, 0.366519, 0.418879,			//right knee, ankle pitch, ankle roll
			0.383972, 0.349066, 0.767945;			//neck pitch, roll, yaw


	qmin << -inf, -inf, -inf, -inf, -inf, -inf,
			-0.383972, -0.680678, -1.02974,                 // torso pitch, roll, yaw
			-1.65806,  0.00000, -0.645772, 		     	//left shoulder pitch ,roll, yaw
			0.0959931, 					//left elbow
			-0.872665, -1.13446, -0.436332, 		//left wrist prosup, pitch, yaw
			-0.767945, -0.296706, -1.37881,			//left hip pitch, roll, yaw
			-2.18166, -0.733038, -0.418879 ,		//left knee, ankle pitch, ankle roll
			-1.65806,  0.00000, -0.645772, 		     	//right shoulder pitch ,roll, yaw
			0.0959931, 					//right elbow
			-0.872665, -1.13446, -0.436332, 		//right wrist prosup, pitch, yaw
			-0.767945, -0.296706, -1.37881,			//right hip pitch, roll, yaw
			-2.18166, -0.733038, -0.418879,			//right knee, ankle pitch, ankle roll
			-0.523599, -0.349066, -0.767945;		//neck pitch, roll, yaw


}

void Model::set_state(double tt, Cvector& in_state, Cvector& in_stated)
{
    time = tt;
    sdfast_model_state( tt, &in_state[0], &in_stated[0] );
}

Cvector Model::get_equivht()
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    sdfast_model_equivht( &tau[0] );
    return tau;
}

Cvector Model::get_comptrq(Cvector& udot)
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    sdfast_model_comptrq( &udot[0],  &tau[0] );
    return tau;
}

Cvector Model::get_fulltrq(Cvector& udot, Cvector& mult)
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    sdfast_model_fulltrq( &udot[0], &mult[0], &tau[0] );
    return tau;
}

Cmatrix Model::get_massmat()
{
    double mmat[AIR_N_U][AIR_N_U];
    sdfast_model_massmat( mmat );
    Cmatrix M = Cmatrix::Zero(AIR_N_U, AIR_N_U);
    for(int i=0;i<AIR_N_U;i++)
        for(int j=0;j<AIR_N_U;j++)
            M(i,j) = mmat[i][j];
    return M;
}

Cvector Model::get_frcmat()
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    sdfast_model_frcmat( &tau[0] );
    return tau;
}

Cvector Model::get_multtrq(Cvector & mult)
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    sdfast_model_multtrq( &mult[0], &tau[0] );
    return tau;
}

Cvector3 Model::get_pos(int body, Cvector3 pt)
{
    Cvector3 loc_cp = zero_v3;
    sdfast_model_pos(body, &pt[0], &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_vel(int body, Cvector3 pt)
{
    // automatically gives the velocity in the global ref frame
    Cvector3 loc_cp = zero_v3;
    sdfast_model_vel(body, &pt[0], &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_acc(int body, Cvector3 pt)
{
    Cvector3 loc_cp = zero_v3;
    sdfast_model_acc(body, &pt[0], &loc_cp[0]);
    return loc_cp;
}

Cmatrix Model::get_orient(int body)
{
    double dircos[3][3];
    sdfast_model_orient(body, dircos);
    return matrix3_convert(dircos);
}

Cvector Model::get_orient_quat(int body)
{
    Cmatrix Orient = get_orient(body);
    return dc2quat(Orient);
}

Cvector3 Model::get_orient_ang(int body)
{
    Cmatrix Orient = get_orient(body);
    return dc2ang(Orient);
}

Cvector3 Model::get_angvel(int body)
{
    // gives the velocity in the local ref frame, we should multiply by body
    Cvector3 loc_cp;
    sdfast_model_angvel(body, &loc_cp[0]);
    sdfast_model_trans(body, &loc_cp[0], -1, &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_angacc(int body)
{
    // gives the velocity in the local ref frame, we should multiply by body
    Cvector3 loc_cp;
    sdfast_model_angacc(body, &loc_cp[0]);
    sdfast_model_trans(body, &loc_cp[0], -1, &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_trans(int frbod, Cvector3& ivec, int tobod)
{
    double loc_cp[3];
    sdfast_model_trans(frbod, &ivec[0], tobod, loc_cp);
    return Cvector3(loc_cp[0], loc_cp[1], loc_cp[2]);
}

Cmatrix Model::get_jacob(int body, Cvector3 pt, constraint_type type)
{
    // gives the velocity in the local ref frame, we should multiply
    double lin[3];
    double rot[3];

    Cmatrix J((type==CT_TRANSLATION || type==CT_ROTATION) ? 3 : 6, AIR_N_U);
    for(int i=0;i<AIR_N_U;i++)
    {
        sdfast_model_rel2cart(i, body, &pt[0], lin, rot);
        if(type==CT_TRANSLATION || type==CT_FULL)
        {
            sdfast_model_trans(body, lin, -1, lin);
            J(0, i) = lin[0];
            J(1, i) = lin[1];
            J(2, i) = lin[2];
        }
        if(type==CT_ROTATION || type==CT_FULL)
        {
            int shift = type==CT_FULL ? 3 : 0;
            sdfast_model_trans(body, rot, -1, rot);
            J(shift+0, i) = rot[0];
            J(shift+1, i) = rot[1];
            J(shift+2, i) = rot[2];
        }
    }
    return J;
}

void Model::set_acc(Cvector &acc)
{
    sdfast_model_setudot(&acc[0]);
}

void  Model::get_mom(Cvector3 &LM, Cvector3 &AM, double * KE)
{
    double ke;
    sdfast_model_mom(&LM[0], &AM[0], &ke);
    if(KE!=NULL)
        *KE = ke;
}

void Model::get_sys(Cvector3 &CM, Cmatrix &ICM, double * MTOT)
{
    double icm[3][3];
    double mtot;
    sdfast_model_sys(&mtot, &CM[0], icm);
    ICM = matrix3_convert(icm);
    *MTOT = mtot;
}

double Model::get_mass()
{
    double mass=0;
    double dm;
    for(int i=0;i<AIR_N_BODY;i++)
    {
        sdfast_model_getmass(i, &dm);
        mass += dm;
    }
    return mass;
}

double Model::get_mass(int body)
{
    double dm;
    sdfast_model_getmass(body, &dm);
    return dm;
}

Cmatrix Model::get_inertia(int body)
{
    double icm[3][3];
    sdfast_model_getiner(body, icm);
    return matrix3_convert(icm);
}

Cvector3 Model::get_cm()
{
    double mass=0;
    Cvector3 cm = zero_v3;
    double pt_cp[3] = {0,0,0};
    double loc_cp[3];
    for(int i=0;i<AIR_N_BODY;i++)
    {
        sdfast_model_getmass(i, &mass);
        sdfast_model_pos(i, pt_cp, loc_cp);
        cm[0] +=  loc_cp[0] * mass;
        cm[1] +=  loc_cp[1] * mass;
        cm[2] +=  loc_cp[2] * mass;
    }
    return cm/get_mass();
}

Cvector3 Model::get_cm_v()
{
    double mass=0;
    Cvector3 cm = zero_v3;
    double pt_cp[3] = {0,0,0};
    double loc_cp[3];
    for(int i=0;i<AIR_N_BODY;i++)
    {
        sdfast_model_getmass(i, &mass);
        sdfast_model_vel(i, pt_cp, loc_cp);
        cm[0] +=  loc_cp[0] * mass;
        cm[1] +=  loc_cp[1] * mass;
        cm[2] +=  loc_cp[2] * mass;
    }
    return cm/get_mass();
}

Cvector3 Model::get_cm_a()
{
    double mass=0;
    Cvector3 cm = zero_v3;
    double pt_cp[3] = {0,0,0};
    double loc_cp[3];
    for(int i=0;i<AIR_N_BODY;i++)
    {
        sdfast_model_getmass(i, &mass);
        sdfast_model_acc(i, pt_cp, loc_cp);
        cm[0] +=  loc_cp[0] * mass;
        cm[1] +=  loc_cp[1] * mass;
        cm[2] +=  loc_cp[2] * mass;
    }
    return cm/get_mass();
}

Cmatrix Model::get_cm_J()
{
    Cmatrix J = Cmatrix::Zero(3, AIR_N_U);
    double pt_cp[3]={0,0,0};
    double lin[3];
    double rot[3];

    double mass = 0;
    double M = get_mass();
    for(int j=0;j<AIR_N_BODY;j++)
    {
        sdfast_model_getmass(j, &mass);
        mass /= M;
        for(int i=0;i<AIR_N_U;i++)
        {
//            if(j>=AIR_BODY_TORSO_X && j<=AIR_BODY_TORSO_Z && i>S_AIR_TORSO_Z && i<S_AIR_L_ARM_SHOULDER_Y) continue;
//            if(j>=AIR_BODY_L_LEG_HIP_Y && j<=AIR_BODY_L_LEG_FOOT_Y && i>S_AIR_L_LEG_FOOT_Y) continue;
//            if(j>=AIR_BODY_R_LEG_HIP_Y && j<=AIR_BODY_R_LEG_FOOT_Y && i>S_AIR_R_LEG_FOOT_Y) continue;
//            if(j>=AIR_BODY_L_ARM_SHOULDER_Y && j<=AIR_BODY_L_ARM_ELBOW_Y && i>S_AIR_TORSO_Z && i<S_AIR_L_ARM_SHOULDER_Y) continue;
//            if(j>=AIR_BODY_R_ARM_SHOULDER_Y && j<=AIR_BODY_R_ARM_ELBOW_Y && i>S_AIR_TORSO_Z && i<S_AIR_R_ARM_SHOULDER_Y) continue;

            sdfast_model_rel2cart(i, j, pt_cp, lin, rot);
            sdfast_model_trans(j, lin, -1, lin);
            J(0, i) += lin[0]*mass;
            J(1, i) += lin[1]*mass;
            J(2, i) += lin[2]*mass;
        }
    }
    return J;
}

void Model::check_consistency(Cvector joint_pos, Cvector joint_vel, Cvector joint_acc)
{
    // check consistency of joint and global variables, jacobians and sdfast functions
    ///////////////////////////// global vars
    // suppose the inertial fixed frame is called i
    // and the base frame is called b
    Cvector OR = ang2quat(Cvector3(0.5,0.8,0.7));
    Cmatrix rot = quat2dc(OR); // base orientation in i
    Cvector3 w(0.1, 0.5, -0.4); // base angular velocity expressed in b.
    Cvector3 v(0.01,0.02,0.03); // base linear velocity in i
    Cvector3 p(0.1,0.3,0.6); // base position in i
    Cvector3 a(-0.3,0.5,0.7); // base acceleration in i
    Cvector3 al(-0.6,0.4,0.9); // base angular acceleration in b.

    // now chose a link in the robot
    int body = 7; //AIR_BODY_L_LEG_KNEE_Y;
    Cvector3 offset_r = zero_v3;

    ///////////////////////////// raw kinematics
    // setting base positions and orientations to zero in sdfast
    // in order to calculate end-effector's (chosen body) variables in base frame
    joint_pos[AIR_N_Q-1] = 1;
    joint_pos.segment(0,6).setZero();
    joint_vel.segment(0,6).setZero();
    joint_acc.segment(0,6).setZero();

//    for(int i=6;i<AIR_N_U;i++)
//    {
//        joint_pos[i] += (double(rand()%1000)/1000.0*2.0-0.5);
//        joint_vel[i] += (double(rand()%1000)/1000.0*2.0-0.5);
//        joint_acc[i] += (double(rand()%1000)/1000.0*2.0-0.5);
//    }

    set_state(0, joint_pos, joint_vel);
    set_acc(joint_acc);

    Cvector3 pos = get_pos(body, offset_r);
    Cvector3 vel = get_vel(body, offset_r);
    Cvector3 acc = get_acc(body, offset_r);
    Cmatrix ang = get_orient(body);
    Cvector3 angvel = get_angvel(body);
    Cvector3 angacc = get_angacc(body);

    ///////////////////////////// complex kinematics
    // setting base positions and orientations to desired values in sdfast
    // in order to calculate end-effector's (chosen body) variables in inertial frame
    joint_pos[AIR_N_Q-1] = OR[3];
    joint_pos.segment(0,6) = vector6(p,OR.segment(0,3));
    joint_vel.segment(0,6) = vector6(v,w);
    joint_acc.segment(0,6) = vector6(a,al);
    set_state(0, joint_pos, joint_vel);
    set_acc(joint_acc);

    cout << endl << "/////////////check consistency//////////////" << endl;

    Cvector3 pos1 = p + rot * pos;
    cout << "pos1 " << pos1.transpose() << endl;
    Cvector3 pos2 = get_pos(body, offset_r);
    cout << "pos2 " << pos2.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 rot1 = dc2ang(rot * ang);
    cout << "rot1 " << rot1.transpose() << endl;
    Cvector3 rot2 = get_orient_ang(body);
    cout << "rot2 " << rot2.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 vel1 = v + rot * vel + rot * skew_symmetric(w) * pos;
    cout << "vel1 " << vel1.transpose() << endl;
    Cvector3 vel2 = get_vel(body, offset_r);
    cout << "vel2 " << vel2.transpose() << endl;
    Cvector3 vel3 = get_jacob(body,offset_r,CT_TRANSLATION) * joint_vel;
    cout << "vel3 " << vel3.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 angvel1 = rot * angvel + rot * w;
    cout << "angvel1 " << angvel1.transpose() << endl;
    Cvector3 angvel2 = get_angvel(body);
    cout << "angvel2 " << angvel2.transpose() << endl;
    Cvector3 angvel3 = get_jacob(body,offset_r,CT_ROTATION) * joint_vel;
    cout << "angvel3 " << angvel3.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 acc1 = a + rot * acc +
                    rot * skew_symmetric(w) * vel * 2 +
                    rot * skew_symmetric(w) * skew_symmetric(w) * pos +
                    rot * skew_symmetric(al) * pos;
    cout << "acc1 " << acc1.transpose() << endl;
    Cvector3 acc2 = get_acc(body, offset_r);
    cout << "acc2 " << acc2.transpose() << endl;

    Cvector copy = joint_acc * 0;
    set_acc(copy);
    Cvector3 acc3 = get_acc(body, offset_r);
    acc3 += get_jacob(body,offset_r,CT_TRANSLATION) * joint_acc;
    set_acc(joint_acc);
    cout << "acc3 " << acc3.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 angacc1 = rot * skew_symmetric(w) * angvel +
                      rot * angacc +
                      rot * al;
    cout << "angacc1 " << angacc1.transpose() << endl;
    Cvector3 angacc2 = get_angacc(body);
    cout << "angacc2 " << angacc2.transpose() << endl;

    set_acc(copy);
    Cvector3 angacc3 = get_angacc(body);
    angacc3 += get_jacob(body,offset_r,CT_ROTATION) * joint_acc;
    set_acc(joint_acc);
    cout << "angacc3 " << angacc3.transpose() << endl;

    cout << "/////////////check consistency//////////////" << endl << endl;

// conclusion:
// in the state vector, pos, vel and acc are in global frame
// in the state vector, OR is global, but w and al are local, in base frame
}
