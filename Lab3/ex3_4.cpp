#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <rw/rw.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;


#define PI 3.14159256


float Tref_nTCP[4][4] = {{1,0,0,1},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

//===========================================================================================================================================


//===========================================================================================================================================

int main()
{
	cout << endl;
	cout << "	=======================================================" << endl;
	cout << " 			FORWARD KINEMATICS" << endl;
	cout << "	=======================================================" << endl;
	cout << endl;
	cout << "This program compute forward kinematics of the UR5 robot in the WorkCell created at exercise 3.3." << endl;
	cout << endl;

//-----------------------------------------------------------------------------------------

	// Forward Kinematics Info

	cout << " 	>> World T reference:" << endl;
	cout << endl;
	for (int u=0; u<4; u++)
	{
		for (int v=0; v<4; v++)
		{
			cout << Tref_nTCP[u][v] << "   ";		
		}
		cout << endl;
	}
	cout << endl;
	cout << "	>> T(base,TCP) = T(base,1)*T(1,2)*...*T(n-1,n)*T(n,TCP)" << endl;
	cout << "	>> Being T(i-1,i) = Tref(i-1,i)*T(qi)." << endl;
	cout << "  " << endl;
	cout << "--------------------------------------------------------------------------" << endl;
	cout << "  " << endl;

//-----------------------------------------------------------------------------------------

	// Loading WorkCell
	
	const std::string wcFile = "/home/student/workspace/workcells/UR5WorkCellFinal/Scene.wc.xml";
	cout << "Opening WorkCell ---> " << wcFile << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;
	// if found, loading workcell
	WorkCell::Ptr wc = WorkCellFactory::load(wcFile);
	cout << "WorkCell contains the following devices:" << endl;
	cout << "  " << endl;
	BOOST_FOREACH(Device::Ptr dvcs, wc->getDevices()){
		cout << ">>  " << dvcs->getName() << endl;
	}
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//-----------------------------------------------------------------------------------------

	// Settings

	cout << "ROBOT STATE:" << endl;
	cout << endl;
	const std::string device_name = "UR-6-85-5-A";
  	rw::models::Device::Ptr device = wc->findDevice(device_name);
  	if(device == nullptr) 
	{
     		 RW_THROW("Device " << device_name << " was not found!");
  	}

  	// Set robot state, q
  	rw::kinematics::State state = wc->getDefaultState();
	rw::math::Q q(6, 0.859, 0.208, -0.825, -0.746,  -1.632, 1.527);
  	device->setQ(q, state);
	cout << "Default joint configuration:" << endl;
	cout << "	>> " << q << endl;
	
	// TCP frame
	rw::kinematics::Frame* tcp_frame = wc->findFrame(device_name + ".TCP");
 	if(tcp_frame == nullptr) 
	{
      		RW_THROW("TCP frame not found!");
 	}
	cout << "TCP frame:" << endl;
	cout << "	>> " << tcp_frame->getName() << endl;
	cout << "State:" << endl;
	cout << "	>> " << tcp_frame->getTransform(state) << endl;
	cout << endl;


	// Joint 0: <RPY> 0 0 0 </RPY> <Pos> 0 0 0 </Pos>
 	 rw::math::Vector3D<> V0(0, 0, 0);
  	rw::math::RPY<> R0(0, 0, 0);
  	rw::math::Transform3D<> T0(V0, R0.toRotation3D());

  	// Joint 1: <RPY> 90 0 90 </RPY> <Pos> 0 0 0.08920 </Pos>
  	rw::math::Vector3D<> V1(0, 0, 0.08920);
  	rw::math::RPY<> R1(90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
  	rw::math::Transform3D<> T1(V1, R1.toRotation3D());

  	// Joint 2: <RPY> 270 0 0 </RPY> <Pos> 0 0.425 0 </Pos>
  	rw::math::Vector3D<> V2(0, 0.425, 0);
 	rw::math::RPY<> R2(270*rw::math::Deg2Rad, 0, 0);
  	rw::math::Transform3D<> T2(V2, R2.toRotation3D());

  	// Joint 3: <RPY> 0 0 0 </RPY> <Pos> -0.39243 0 0 </Pos>
  	rw::math::Vector3D<> V3(-0.39243, 0, 0);
  	rw::math::RPY<> R3(0, 0, 0);
  	rw::math::Transform3D<> T3(V3, R3.toRotation3D());

  	// Joint 4: <RPY> 270 0 90 </RPY> <Pos> 0 0 0.109 </Pos>
  	rw::math::Vector3D<> V4(0, 0, 0.109);
  	rw::math::RPY<> R4(270*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
  	rw::math::Transform3D<> T4(V4, R4.toRotation3D());
  
  	// Joint 5: <RPY> 0 0 270 </RPY> <Pos> 0 0 0.093 </Pos>
  	rw::math::Vector3D<> V5(0, 0, 0.093);
  	rw::math::RPY<> R5(0, 0, 270*rw::math::Deg2Rad);
  	rw::math::Transform3D<> T5(V5, R5.toRotation3D());


	cout << "Transform matrices for all i-th joint frames of the robot:" << endl;
	cout << endl;
	cout << "  Joint 0:" << endl;
	cout << endl;
	cout << "	>> " << T0 << endl;
	cout << endl; 
	cout << "  Joint 1:" << endl;
	cout << endl;
	cout << "	>> " << T1 << endl;
	cout << endl; 
	cout << "  Joint 2:" << endl;
	cout << endl;
	cout << "	>> " << T2 << endl;
	cout << endl; 
	cout << "  Joint 3:" << endl;
	cout << endl;
	cout << "	>> " << T3 << endl;
	cout << endl; 
	cout << "  Joint 4:" << endl;
	cout << endl;
	cout << "	>> " << T4 << endl;
	cout << endl; 
	cout << "  Joint 5:" << endl;
	cout << endl;
	cout << "	>> " << T5 << endl;
	cout << endl; 

	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;
	
	

//-----------------------------------------------------------------------------------------
		
	// Output
	cout << "RESULT" << endl;
	cout << endl;
	cout << "Forward Kinematics [T(base, TCP)] of " << device_name << " at default state:" << endl;
	cout << endl;
	rw::math::Transform3D<> baseTtool_rw = device->baseTframe(tcp_frame, state);
	cout << "	>> " << baseTtool_rw << endl; 
	cout << endl;
	cout << endl;
	cout << endl;
	cout << "				---------------------------------------------------------" << endl;
	cout << "				 Carlos Viescas Huerta | SDU Robotics | RoVi1 - Robotics" << endl;
	cout << "				---------------------------------------------------------" << endl;
	return 0;

}
