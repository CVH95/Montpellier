// PROGRAMMING EXERCISE 4.4 - ROVI1 - ROBOTICS

// @author: CARLOS VIESCAS HUERTA

#include <iostream>
#include <fstream> 
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>

using namespace std;
using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

//=========================================================================================================================================

// FUNTIONS

void theoryScript()
{
	ofstream fs;
	fs.open("/home/student/workspace/Robotics_Projects/Lecture8/genfiles/theory_script.txt");
	fs << "		======================================================" << endl;
	fs << "	 		 JACOBIAN FOR TOOL MOUNTED CAMERA" << endl;
	fs << "		======================================================" << endl;
	fs << endl;
	fs << "THEORY:" << endl;
	fs << "	  	1. Jacobians and visual servoing for a tool mounted camera." << endl;
	fs << "			>> Assume camera coordinate frame = Fcam;    ||    Focal length of the camera --> f [pixels]." << endl;
	fs << "			>> Consider point P in CAMERA coordinates [3D] --> P = (x,y,z,1)' referred to Fcam" << endl;
	fs << "			>> IMAGE coordinates (u, v) [2D] of point P --> u = f*x/z | v = f*y/z" << endl;
	fs << "		2. Assumptions for this model:" << endl;
	fs << "			>> Pinhole camera model with the same focal length f in both x and y directions and the origin of the uv coordinate system has been placed at the optical axis." << endl;
	fs << "			>> u = v = 0 --> Center of the image." << endl;
	fs << " 		>> " << endl;
	fs << " 	3. Image Jacobian." << endl;
	fs << "			>> It is used for computing the relationship between displacements of the robot and the corresponding displacements of the image coordinates when point P is at rest w.r.t. WOLRD FRAME" << endl;
	fs << " 		>> P->base = T(base, cam)* P->cam" << endl;
	fs << "			>> When there is displacement:" << endl;
	fs << " 			du (derivative of coord. u) = (f/z)*dx - (f*x/z²)*dz" << endl;
	fs << " 			dv (derivative of coord. v) = (f/z)*dy - (f*y/z²)*dz" << endl;
	fs << "			>> Assuming small positional displacement dp and angular displacment dw:" << endl;
	fs << " 			r->base = T(base, cam)[I+R(dw) | dp] * (P + dr)   --->   where dr is displacement of point r in the camera frame due to camera movement and for R(dw) see Eqn 4.1 in notes" << endl;
	fs << "			>> [du dv]' = Jimage * du  ---> du = infinitesimal displacement vector (analogous to du = J_q * dq, ex 4_2)" << endl;
	fs << "						| -(f/z)  0  u/z  u*v/f  -(f²+u²)/f  v|" << endl;
	fs << " 		>> Jimage (2x6) =       |                                     |" << endl;
	fs << "						| 0 -(f/z)  v/z  -(f²+v²)/f  -u*v/f  u|" << endl;
	fs << "		4. Zimage(q) for a tool-mounted camera." << endl;
	fs << "			>> Zimage(q) (2xn) = Jimage * S(q) * J(q) where:" << endl;
	fs << "						n = number of joints." << endl;
	fs << "						J(q) = Jacobian(base, tool) assuming Fcam = Ftool and taken from du = J(q)*dq." << endl;
	fs << "						S(q) = matrix converting du vector to base frame (du->base.)" << endl;
	fs << endl;
	fs.close();
}

//----------------------------------------------------------------------------

void printKinTree(Frame& printFrame, const State& printState, const Transform3D<>& parentTransform, int level)
{

	const Transform3D<> transform = parentTransform * printFrame.getTransform(printState);
	cout << level << ":  " << "|Frame name: " << printFrame.getName() << "|   " << "|Default State at " << transform.P() << "|" << endl;
	cout << endl;
	BOOST_FOREACH(Frame& child, printFrame.getChildren(printState))
	{
		cout << "            ";
		printKinTree(child, printState, transform, level+1);
	}	

}

//-----------------------------------------------------------------------------

// Check for collisions at a given state (Q).
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) 
{
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) 
	{
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) 
		{
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

//-------------------------------------------------------------------------------

// Function that calculates (u, v) image coordinates of a point given in Fcam coordinates. 
vector<int> Image_Point(int x, int y, int z, int f)
{
	// Point (u, v)
	vector<int> image_point;
	
	// Coordinate u:
	int u = f*x/z;
	image_point.push_back(u);
	// Coordinate v:
	int v = f*y/z;
	image_point.push_back(v);
	
	return image_point;
}

//-------------------------------------------------------------------------------

// Funtion that calculates the image Jacobian Jimage for a certain point P.
// Inputs 	---->		P(x,y,z)->Fcam
//				P(u,v)->image 
//				f == focal length [pixels]
// Implementation of the formula given at Robotics Notes page 51.
rw::math::Jacobian image_Jacobian(int x, int y, int z, int f, int u, int v)
{
	// Constructor
	rw::math::Jacobian Jimage(2,6);

	// Row u:
	Jimage(0,0) = (double) -(f/z);
	Jimage(0,1) = (double) 0;
	Jimage(0,2) = (double) u/z;
	Jimage(0,3) = (double) (u*v)/f;
	Jimage(0,4) = (double) -(((f*f)+(u*u))/f);
	Jimage(0,5) = (double) v;

	// Row v:
	Jimage(1,0) = (double) 0;
	Jimage(1,1) = (double) -(f/z);
	Jimage(1,2) = (double) v/z;
	Jimage(1,3) = (double) ((f*f)+(v*v))/f;
	Jimage(1,4) = (double) -((u*v)/f);
	Jimage(1,5) = (double) -u;

	return Jimage;
}

//--------------------------------------------------------------------------------

rw::math::Jacobian duToBase(rw::math::Transform3D<double> T_0)
{
	// Defining S_q container.
	rw::math::Jacobian S_q(6, 6);
	
	// Defining Rotation matix from base to tool
	rw::math::Rotation3D<double> R_bt = T_0.R();
	// Transposing R_bt ------> The transpose of a rotation matrix is the same as the inverse.
	rw::math::Rotation3D<double> R_bt_T = rw::math::inverse(R_bt);
	
	// Building S_q

	// First quadrant of the matrix --> S_q(0,0) to S_q(2,2) = R_bt_T(0,0) to R_bt_T(2,2)
	for(int i=0; i<=2; i++)
	{
		for(int j=0; j<=2; j++)
		{
			S_q(i,j) = R_bt_T(i,j);		
		}
	}
	
	// Second quadrant of the matrix --> S_q(0,3) to S_q(2,5) = 0 all
	for(int k=0; k<=2; k++)
	{
		for(int l=3; l<=5; l++)
		{
			S_q(k,l) = 0;		
		}
	}
	
	// Third quadrant of the matrix --> S_q(3,3) to S_q(5,5) = R_bt_T(0,0) to R_bt_T(2,2)
	for(int m=3; m<=5; m++)
	{
		for(int n=3; n<=5; n++)
		{
			S_q(m,n) = R_bt_T(m-3,n-3);		
		}
	}
	
	// Fourth quadrant of the matrix --> S_q(3,0) to S_q(5,2) = 0 all
	for(int p=3; p<=5; p++)
	{
		for(int q=0; q<=2; q++)
		{
			S_q(p,q) = 0;		
		}
	}

	return S_q;
}

//--------------------------------------------------------------------------------

//Zimage_q (2xn=6)= Jimage_r(2x6) * S_q(6x6) * J_q(6xn=6)"
rw::math::Jacobian compute_Z_image_q(rw::math::Jacobian Jimage, rw::math::Jacobian S_q, rw::math::Jacobian J_q)
{
	rw::math::Jacobian Zimage_q(2,6);
	rw::math::Jacobian J1(2,6);
	
	// Calculate first J1 container
	J1 = Jimage * S_q;
	
	// Calculate Zimage_q
	Zimage_q = J1 * J_q;

	return Zimage_q;
	
}

//=========================================================================================================================================

//MAIN

int main(int argc, char** argv) 
{

	cout << endl;
	cout << "				----------------------------------------------------------" << endl;
	cout << "					     JACOBIAN FOR TOOL MOUNTED CAMERA" << endl;
	cout << "				----------------------------------------------------------" << endl;
	cout << endl;
	cout << endl;
	cout << endl;

//------------------------------------------------------------------------------------------------------------

	//LOADING WORKCELL

	const string wcFile = "/home/student/workspace/workcells/URInterpolate/Scene.wc.xml"; 
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
	// Define state variable
	State state = wc->getDefaultState();
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;
	cout << "KINEMATIC TREE:" << endl;
	// Display kinematic tree for given workcell to see all frame names.
	printKinTree(*wc->getWorldFrame(), state, Transform3D<>::identity(), 0);
	cout << "------------------------------------------------------------------------------------" << endl;

//------------------------------------------------------------------------------------------------------------

	// INITIAL SETTINGS OF UR5 ROBOT
	
	// Find device UR-6-85-5-A
	
	string deviceName = "UR-6-85-5-A";
	
	Device::Ptr device = wc->findDevice("UR-6-85-5-A");
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	
	// Define initial joint - configuration
	//rw::math::Q q_init = device->getQ(state);
	Q q_start(6,1,2,1,-1,1,1);
	// Set robot position	
	device->setQ(q_start, state);
	// Check for collisions	
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 
	if (!checkCollisions(device, state, detector, q_start))
		{return 0;}

	cout << endl;
	cout << "Robot device: <<" << deviceName << ">> " << endl;
	cout << "	>> Number of joints: 6" << endl;
	cout << "Initial configuration:" << endl;
	cout << "	>> q_start = (" << q_start[0] << "," << q_start[1] << "," << q_start[2] << "," << q_start[3] << "," << q_start[4] << "," << q_start[5] <<"})" << endl;
	cout << endl;
	cout << "Succesfully set to initial configuration." << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//--------------------------------------------------------------------------------------------------------------

	// Point 
	
	cout << "1ST PART: CALCULATING IMAGE POINTS AND IMAGE JACOBIANS" << endl;
	cout << endl;
	cout << "Point r given in coordinates (x,y,z,1)' referred to frame Fcam of a tool-mounted camera" << endl;
	cout << "Values given by the exercise:" << endl;
	cout << "	>> r(1,1,4)->Fcam;" << endl;
	cout << "	>> f = 500 pixels;" << endl;
	cout << endl;

	// Defining values 
	int x, y, z, f;
	x = 1;
	y = 1;
	z = 4;
	f = 500;

	// Calculating u,v
	vector<int> r_img = Image_Point(x, y, z, f);
	int u = r_img[0];
	int v = r_img[1];
	
	// Displaying u,v 
	cout << "	>> Image coordinates of point r:" << endl;
	cout << "r(u, v) = (" << u << ", " << v << ")" << endl;
	cout << endl;
	

	// Calculating image Jacobian
	rw::math::Jacobian Jimage_r = image_Jacobian(x, y, z, f, u, v);
	
	// Displaying Jacobian
	cout << "	>> Image Jacobian of point r (2x6 matrix):" << endl;
	cout << "Jimage_r = " << endl;
	cout << endl;
	cout << Jimage_r << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//-----------------------------------------------------------------------------------------------------------

	// On UR5 Robot

	cout << "2ND PART: CALCULATING Zimage(Q) FOR UR5 ROBOT" << endl;
	cout << endl;
	cout << " 	>> We assume that the camera is aligned with the tool frame of the robot  --->  Ftool = Fcam." << endl;
	cout << "	>> Image based visual servoing (for point r)  -->  Zimage_q (2xn=6)= Jimage_r(2x6) * S_q(6x6) * J_q(6x6)" << endl;
	cout << endl;
	const string tool_name = "Tool";
	rw::kinematics::Frame* tool_frame =  wc->findFrame("Tool");
	if(tool_frame == nullptr) {
        RW_THROW("Tool frame not found!");
    	}

	// To compute the Jacobian J_q we need the T matrix T_0(base, tool)
	// Get Transform Matrix T_0
	rw::math::Transform3D<double> T_0 = device->baseTframe(tool_frame, state);
	// Get Jacobian J_q
	rw::math::Jacobian J_q = device->baseJframe(tool_frame, state);

	// Calculate S_q
	rw::math::Jacobian S_q = duToBase(T_0);

	// Calculate Zimage_q
	rw::math::Jacobian Zimage_q = compute_Z_image_q(Jimage_r, S_q, J_q);
	
	// Displaying
	cout << "	>> Transform matrix T(base, tool){q_start}:" << endl;
	cout << endl;
	cout << "T_0 = ";
	cout << T_0 << endl;
	cout << endl;
	cout << "	>> Jacobian J_q {q_start} (6x6):" << endl;
	cout << "J_q = " << endl;
	cout << endl;
	cout << J_q << endl;
	cout << endl;	
	cout << "	>> du To du->base Transformation matrix S_q (6x6):" << endl;
	cout << "S_q = " << endl;
	cout << endl;
	cout << S_q << endl;
	cout << endl;	
	cout << "	>> Calculated matrix operator Zimage_q (2x6) for image based visual servoing:" << endl;
	cout << "Zimage_q = " << endl;
	cout << endl;
	cout << Zimage_q << endl;
	cout << endl;	
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//-----------------------------------------------------------------------------------------------------------

	theoryScript();
	cout << "Programm done. Checkout ../genfiles/ directory." << endl;
	cout << endl;
	cout << endl;
	cout << endl;
	cout << "				---------------------------------------------------------" << endl;
	cout << "				 Carlos Viescas Huerta | SDU Robotics | RoVi1 - Robotics" << endl;
	cout << "				---------------------------------------------------------" << endl;
	return 0;

}
