// PROGRAMMING EXERCISE 4.2 - ROVI1 - ROBOTICS

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

//=========================================================================================================================

// FUNCTIONS 

// Display kinematic tree of the workcell on command line.
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

//··········································································································

// Theory script (personal)

void theory_script()
{
	ofstream fs;
	fs.open("/home/student/workspace/Robotics_Projects/Lecture6/genfiles/Jacobian_based_IK_small_displacements.txt");
	fs << "		==============================================================================" << endl;
	fs << "	 		 JACOBIAN BASED INVERSE KINEMATICS FOR SMALL DISPLACEMENTS" << endl;
	fs << "		==============================================================================" << endl;
	fs << endl;
	fs << "THEORY:" << endl;
	fs << endl;
	fs << "		Inputs:" << endl;
	fs << "			>> Configuration vector Q;" << endl;
	fs << "			>> T(base, TCP){Q} = T_0;" << endl;
	fs << "			>> T(base, TCP){position_desired} = T_f;" << endl;
	fs << "		Internal parameters:" << endl;
	fs << " 		>> Infinitesimal positional change --> dp = p_f - p_0 ····· p ~ T_0 <p_0, RPY_0>;" << endl;
	fs << " 		>> Infinitesimal angular displacement from RPY_0 to RPY_f --> dw --> RPY_f = RPY(dw) * RPY_0 ~~~ RPY(dw) = RPY_f * RPY_0';" << endl;
	fs << "			>> dw = (1/2) * vector{RPY(dw)_32 - RPY(dw)_32, RPY(dw)_13 - RPY(dw)_31, RPY(dw)_21 - RPY(dw)_12}" << endl;
	fs << "			>> dq <dp, dw> " << endl;
	fs << "			>> J(q) * dq = [du] ········ du = vector{dp | dw} = W(T_0, T_f)" << endl;
	fs << "		Algorithm:" << endl;
	fs << "			1. du = W(T_0, T_f);" << endl;
	fs << "			2. while (abs(du) > eps) {" << endl;
	fs << " 			3. Compute Jacob(Q) = J_q;" << endl;
	fs << "				4. Solve J_q * dq = du;" << endl;
	fs << " 			5. Update Q --> Q (new) := Q + dq;" << endl;
	fs << " 			6. Update T(base, TCP){Q} --> T_0 (new) := T(base, TCP){Q (new)};" << endl;
	fs << " 			7. Update du --> du = W(T_0 (new), T_f);" << endl;
	fs << endl;
	fs << "------------------------------------------------------------------------------------" << endl;
	fs << endl;
	fs.close();
}

//·············································································································

// Check for collisions at a given state (Q).
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

//·············································································································

// This function calculates delta U as in Equation 4.13. The output class is a velocity screw as that is a 6D vector with a positional and rotational part
// What a velocity screw really is is not important to this class. For our purposes it is only a container.

rw::math::VelocityScrew6D<double> calculateDeltaU(const rw::math::Transform3D<double>& baseTtool, const rw::math::Transform3D<double>& baseTtool_desired) 
{
    // Calculate the positional difference, dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate the rotational difference, dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * rw::math::inverse(baseTtool.R()));

    return rw::math::VelocityScrew6D<double>(dp, dw);
}


//·············································································································

// The inverse kinematics algorithm needs to know about the device, the tool frame and the desired pose. These parameters are const since they are not changed by inverse kinematics
// We pass the state and the configuration, q, as value so we have copies that we can change as we want during the inverse kinematics.

rw::math::Q algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                       const rw::math::Transform3D<double> baseTtool_desired, rw::math::Q q, const double epsilon) 
{
    // We need an initial base to tool transform and the positional error at the start (deltaU) 
    rw::math::Transform3D<> baseTtool = device->baseTframe(tool, state);
    rw::math::VelocityScrew6D<double> deltaU = calculateDeltaU(baseTtool, baseTtool_desired);

    // This epsilon is the desired tolerance on the final position.
    
    while(deltaU.norm2() > epsilon) {
        rw::math::Jacobian J = device->baseJframe(tool, state); // This line does the same as the function from Programming Exercise 4.1
	// We need the inverse of the jacobian. To do that, we need to access the Eigen representation of the matrix.
	// For information on Eigen, see http://eigen.tuxfamily.org/.
	rw::math::Jacobian Jinv(J.e().inverse());

	// In RobWork there is an overload of operator* for Jacobian and VelocityScrew that gives Q
	// This can also manually be done via Eigen as J.e().inverse() * deltaU.e()
	// Note that this approach only works for 6DOF robots. If you use a different robot, you need to use a pseudo inverse to solve the equation J * deltaQ = deltaU 
	rw::math::Q deltaQ = Jinv*deltaU;
	
	// Here we add the change in configuration to the current configuration and move the robot to that position.
        q += deltaQ;
        device->setQ(q, state);

	// We need to calculate the forward dynamics again since the robot has been moved
        baseTtool = device->baseTframe(tool, state); // This line performs the forward kinematics (Programming Exercise 3.4)

	// Update the cartesian position error
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    }
    return q;
}


//=========================================================================================================================

// MAIN

int main(int argc, char** argv) 
{
	cout << endl;
	cout << "				----------------------------------------------------------" << endl;
	cout << "				JACOBIAN BASED INVERSE KINEMATICS FOR SMALL DISPLACEMENTS" << endl;
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

//-------------------------------------------------------------------------------------------------------------

	// INITIAL SETTINGS
	
	// Find device UR-6-85-5-A
	
	string deviceName = "UR-6-85-5-A";
	
	Device::Ptr device = wc->findDevice("UR-6-85-5-A");
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	
	// Find tool frame
	const string tool_name = "Tool";
	rw::kinematics::Frame* tool_frame =  wc->findFrame("Tool");
	if(tool_frame == nullptr) {
        RW_THROW("Tool frame not found!");
    	}

	// Define initial joint - configuration
	//rw::math::Q q_init = device->getQ(state);
	Q initial(6,1.198,-1.166,0.357,-1.213,-1.230,1.332);
	// Set robot position	
	device->setQ(initial, state);
	// Check for collisions	
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 
	if (!checkCollisions(device, state, detector, initial))
		{return 0;}
	cout << endl;
	cout << "Tool frame found succesfully." << endl;	
	
	cout << endl;
	cout << "Robot device: <<" << deviceName << ">> succesfully set to initial configuration:" << endl;
	cout << "	Q = (" << initial[0] << "," << initial[1] << "," << initial[2] << "," << initial[3] << "," << initial[4] << "," << initial[5] <<"})" << endl;
	cout << endl;

	// Get Transform Matrix T_0
	rw::math::Transform3D<double> T_0 = device->baseTframe(tool_frame, state);
	// Get Jacobian J_q
	rw::math::Jacobian J_q = device->baseJframe(tool_frame, state);

	cout << "Transform matrix T(base, tool){Q} =" << endl;
	cout << endl;
	cout << "	>> " << T_0 << endl;
	cout << endl;
	cout << "Jacobian J_q =" << endl;
	cout << endl;
	cout << J_q << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//--------------------------------------------------------------------------------------------------------------

	// BUILDING PARAMETERS

	cout << "Building parameters for desired end position." << endl;
	cout << endl;
	// Small displacement dp
	// dp = p_f - p_0  ······ p --> position vector ~ T<p, RPY>.
	const double epsilon = 1e-4;
	rw::math::Vector3D<double> dp(epsilon, epsilon, epsilon);
	rw::math::Vector3D<double> p_0 = T_0.P();
	rw::math::Vector3D<double> p_f = p_0 + dp;
	rw::math::Rotation3D<double> RPY_0 = T_0.R();

	// Choose T_f by adding dp to T_0's positional vector
	rw::math::Transform3D<double> T_f(p_f, RPY_0);

	cout << "	>> Small displacement vector dp = " << dp << endl;
	cout << "	>> p_f = p_0 + dp = " << p_f << endl;
	cout << "	>> Rotation matrix RPY_0 = RPR_f = " << RPY_0 << endl;
	cout << "Transform matrix T(base, tool){desired position} =" << endl;
	cout << "	>> " << T_f << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//--------------------------------------------------------------------------------------------------------------

	// ALGORITHM 1
	
	cout << "Inverse kinematics for Q_new:" << endl;
	cout << endl;
	// Apply algorithm 1
    	rw::math::Q q_final = algorithm1(device, state, tool_frame, T_f, initial, epsilon);
	cout << "	>> Q_new = (" << q_final[0] << "," << q_final[1] << "," << q_final[2] << "," << q_final[3] << "," << q_final[4] << "," << q_final[5] <<"})" << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//--------------------------------------------------------------------------------------------------------------

	// ERROR CHECKING
	
	cout << "Error checking:" << endl;
	cout << endl;
	
	cout << "Start position: \n";
    	cout << "	>> Position: " << T_0.P() << endl;
	cout << "	>> Rotation: " << T_0.R() << "\n\n";

    	device->setQ(q_final, state);
    	rw::math::Transform3D<> T_end = device->baseTframe(tool_frame, state);
    	cout << "Actual end position: \n";
    	cout << "	>> Position: " << T_end.P() << endl; 
	cout << "	>> Rotation: " << T_end.R() << "\n\n";
	
    	cout << "Desired end position: \n";
    	cout << "	>> Position: " << T_f.P() << endl;
	cout << "	>> Rotation: " << T_f.R() << "\n\n";

    	cout << "Desired length of change in position: " << dp.norm2() << "\n\n";

    	cout << "Errors between desired and actual positions: \n";
    	double dp_length = (T_f.P() - T_end.P()).norm2();
    	rw::math::EAA<double> dw(T_f.R() * T_end.R().inverse());
    	double angle = dw.angle();
    	double axis_length = dw.axis().norm2();
    	cout << "	>> Position: " << dp_length << endl;
    	cout << "	>> Rotation angle: " << angle << endl;
    	cout << "	>> Length of rotation axis: " << axis_length << endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;

//--------------------------------------------------------------------------------------------------------------

	theory_script();

	cout << "Program done. Check genfiles/Jacobian_based_IK_small_displacements.txt to see output file." << endl;
	cout << endl;
	cout << endl;
	cout << endl;
	cout << "				---------------------------------------------------------" << endl;
	cout << "				 Carlos Viescas Huerta | SDU Robotics | RoVi1 - Robotics" << endl;
	cout << "				---------------------------------------------------------" << endl;
	return 0;
}
	
