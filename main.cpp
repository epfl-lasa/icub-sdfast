#include "Model.hpp"
using namespace std;

int main()
{

    Model model;

    model.init();

    
    Cvector ref_pos = Cvector::Zero(AIR_N_Q);

    ref_pos[9] = -29.79 ;    //shoulder pitch,
    ref_pos[10] = 29.79 ;    //shoulder roll,
    ref_pos[12] = 44.98 ;    // l_elbow
    ref_pos[15] = 25.00 ;    // l_wrist_yaw

    ref_pos[22] = -29.79 ;
    ref_pos[23] = 29.79 ;
    ref_pos[25] = 44.98 ;
    ref_pos[28] = 25.00 ;

    Cvector ref_vel = Cvector::Zero(AIR_N_U);
    Cvector ref_acc = Cvector::Zero(AIR_N_U);
    model.set_state(0, ref_pos, ref_vel);


    std::ofstream jacobian_mat ;
    jacobian_mat.open( "jacobian_mat.txt", std::ofstream::out | std::ofstream::app );


   model.check_consistency(Cvector::Random(AIR_N_Q),Cvector::Random(AIR_N_U),Cvector::Random(AIR_N_U));


   Cmatrix jac_tr = model.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION);
   Cmatrix jac_rot = model.get_jacob(11, Cvector3(0,0,0), CT_ROTATION);

/// print desired elements
   for(unsigned int i = 0; i < 16; ++i)
   {
	   jacobian_mat << jac_tr(0,i) << "  "   ;
   }

   jacobian_mat << endl << endl ;
   for(unsigned int i = 0; i < 16; ++i)
      {
   	   jacobian_mat << jac_tr(1,i) << "  "   ;
      }
   jacobian_mat << endl <<endl ;

   for(unsigned int i = 0; i < 16; ++i)
      {
   	   jacobian_mat << jac_tr(2,i) << "  "   ;
      }
   jacobian_mat << endl <<endl <<endl <<endl <<endl ;
/////////////////
   for(unsigned int i = 0; i < 16; ++i)
      {
   	   jacobian_mat << jac_rot(0,i) << "  "   ;
      }

      jacobian_mat << endl << endl ;
      for(unsigned int i = 0; i < 16; ++i)
         {
      	   jacobian_mat << jac_rot(1,i) << "  "   ;
         }
      jacobian_mat << endl <<endl ;

      for(unsigned int i = 0; i < 16; ++i)
         {
      	   jacobian_mat << jac_rot(2,i) << "  "   ;
         }
/// end of print


   cout << model.get_jacob(11, Cvector3(0,0,0), CT_TRANSLATION) << endl << endl;


   cout << model.get_jacob(11, Cvector3(0,0,0), CT_ROTATION) << endl << endl;

 //  cout << model.get_massmat() << endl;

   cout << model.get_mass() << endl;

    return 0;
}
