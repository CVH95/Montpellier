// Exercise 4.3


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

//-----------------------------------------------------------------------------------------------

rw::math::VelocityScrew6D<double> calculateDeltaU(const rw::math::Transform3D<double>& baseTtool, const rw::math::Transform3D<double>& baseTtool_desired) 
{
    // Calculate the positional difference, dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate the rotational difference, dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * rw::math::inverse(baseTtool.R()));

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

//-------------------------------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

// This function returns -1 if val is negative, +1 if val is positive
// and 0 if val is 0
int signum(double val) {
    return (0 < val) - (val < 0);
}


//----------------------------------------------------------------------------


// This function converts a rotation matrix to EAA using the method
// detailed in Section 4.5 of the robotics notes
rw::math::EAA<double> calculateEAAFromR(const rw::math::Rotation3D<double>& R) {
    // Initialize empty axis and angle
    rw::math::Vector3D<double> axis(0.0, 0.0, 0.0);
    double angle = 0.0;

    // Calculate the trace of the rotation matrix as this is used in
    // different parts of the conversion
    double tr = R(0,0) + R(1,1) + R(2,2);

    // Tolerance for special cases close to 0 and pi
    double epsilon = 1e-6;

    // The betas for the close to pi case. This vector is also the
    // direction of the axis for the normal and close to 0 case
    rw::math::Vector3D<double> beta(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));

    // This value is used to check if we are close to 0 or pi
    double x = 0.5*beta.norm2();

    if(x >= epsilon) {
        // Normal case
        // See Eq. 4.17
        angle = std::acos((tr - 1)/2);
        axis = rw::math::normalize(beta);
    } else if(x < epsilon && ((tr - 1) > 0)) {
        // Theta approximately 0
        // See Eq. 4.19
        angle = std::acos((tr -1)/2);
        axis = 0.5*beta;
    } else if(x < epsilon && ((tr - 1) < 0)) {
        // Theta approximately pi
        // Calculate the precise angle
        angle = rw::math::Pi - beta.norm2()/2;

        // Find the alphas
        rw::math::Vector3D<double> alpha;
        if(angle == rw::math::Pi) {
            // If the angle is equal to pi, all the alphas are 1
            alpha(0) = alpha(1) = alpha(2) = 1;
        } else {
            // Otherwise calculate the alphas
            // Find the index of the largest beta
            // Here the Eigen library is used to find the maximum
            // element of a matrix.
            // Our vector is a column vector, i.e. a matrix with 3
            // rows and 1 column 
            Eigen::Vector3d::Index maxRow, maxCol; // maxCol needs to
                                                   // be defined but
                                                   // is not used
            // maxCoeff returns the largest element of the vector, but
            // that value is not needed so we just throw it away
            beta.e().maxCoeff(&maxRow, &maxCol);

            // Fill in the alphas as detailed in the Robotics notes
            for(unsigned int i = 0; i < 3; ++i) {
                if(i == maxRow) {
                    alpha(i) = signum(beta(i));
                } else {
                    alpha(i) = signum(R(i,maxRow) - R(maxRow,i));
                }
            }
        }

        // Calculate the axis as in Eq. 4.20
        for(unsigned int i = 0; i < 3; ++i) {
            axis(i) = alpha(i)*std::sqrt(1 + R(i,i));
        }
        axis /= std::sqrt(3 + tr);
    }

    return rw::math::EAA<double>(axis, angle);
}


//-------------------------------------------------------------------------------


rw::math::Rotation3D<double> calculateRFromEAA(const rw::math::EAA<double>& eaa) {
    // This function directly implements Eq. 4.15
    double C = std::cos(eaa.angle());
    double S = std::sin(eaa.angle());
    double v1 = eaa.axis()[0];
    double v2 = eaa.axis()[1];
    double v3 = eaa.axis()[2];
    return rw::math::Rotation3D<double>(
        v1*v1*(1 - C) + C   , v1*v2*(1 - C) - v3*S, v1*v3*(1 - C) + v2*S,
        v1*v2*(1 - C) + v3*S, v2*v2*(1 - C) + C   , v2*v3*(1 - C) - v1*S,
        v1*v3*(1 - C) - v2*S, v2*v3*(1 - C) + v1*S, v3*v3*(1 - C) + C
        );
}

//=========================================================================================================================



int main(int argc, char** argv) 
{
	cout << endl;
	cout << "				----------------------------------------------------------" << endl;
	cout << "				      ROTATION MATRICES IN THE FORM OF 'AXIS + ANGLE'" << endl;
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

	cout << endl;
	cout << "Tool frame found succesfully." << endl;	
	
	rw::math::Q initial = device->getQ(state);
	cout << endl;
	cout << "Robot device: <<" << deviceName << ">> default configuration:" << endl;
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

	// T_0, T_f;
	
    	rw::math::Rotation3D<double> r1 = T_0.R();
	rw::math::Rotation3D<double> r2 = T_0.R();

	rw::math::EAA<double> eaa1 = calculateEAAFromR(r1);
        
   	std::cout << "Normal case test: " << std::endl;
	cout << endl;
    	std::cout << "		>> Rotation matrix:" << r1 << std::endl;
    	std::cout << "		>> EAA axis:" << eaa1.axis() << "  |  EAA angle: " << eaa1.angle() << std::endl;
    	std::cout << "		>> Rotation matrix from EAA: " << calculateRFromEAA(eaa1) << std::endl;
    	std::cout << "		>> RobWork axis: " << rw::math::EAA<double>(r1).axis() << "  |  RobWork angle: " << rw::math::EAA<double>(r1).angle() << std::endl << std::endl;
    	cout << endl;
    

    	rw::math::EAA<double> eaa2 = calculateEAAFromR(r2);
        
    	std::cout << "Angle = pi test: " << std::endl;
	cout << endl;
    	std::cout << "		>> Rotation matrix: " << r2 << std::endl;
    	std::cout << "		>> EAA axis: " << eaa2.axis() << "  |  EAA angle: " << eaa2.angle() << std::endl;
    	std::cout << "		>> Rotation matrix from EAA: " << calculateRFromEAA(eaa2) << std::endl;
    	std::cout << "		>> RobWork axis: " << rw::math::EAA<double>(r2).axis() << "  |  RobWork angle: " << rw::math::EAA<double>(r2).angle() << std::endl << std::endl;
    	cout << endl;
    
    	// This rotation matrix has angle approx 0
    	rw::math::EAA<double> test3(rw::math::EAA<double>(r1).axis(), 10e-6);
    	rw::math::EAA<double> eaa3 = calculateEAAFromR(test3.toRotation3D());
        
    	std::cout << "Angle approx 0 test: " << std::endl;
	cout << endl;
    	std::cout << "		>> Rotation matrix: " << test3.toRotation3D() << std::endl;
    	std::cout << "		>> EAA axis : " << eaa3.axis() << "  |  EAA angle: " << eaa3.angle() << std::endl;
    	std::cout << "		>> Rotation matrix from EAA: " << calculateRFromEAA(eaa3) << std::endl;
    	std::cout << "		>> RobWork axis: " << test3.axis() << "  |  RobWork angle: " << test3.angle() << std::endl << std::endl;
	cout << endl;

    	// This rotation matrix has angle close to pi
   	rw::math::EAA<double> test4(rw::math::Vector3D<double>(1,0,0), rw::math::Pi - 10e-4);
    	rw::math::EAA<double> eaa4 = calculateEAAFromR(test4.toRotation3D());
        
    	std::cout << "Angle approx pi test: " << std::endl;
	cout << endl;    	
	std::cout << "		>> Rotation matrix: " << test4.toRotation3D() << std::endl;
    	std::cout << "		>> EAA axis: " << eaa4.axis() << "  |  EAA angle: " << eaa4.angle() << std::endl;
    	std::cout << "		>> Rotation matrix from EAA: " << calculateRFromEAA(eaa4) << std::endl;
    	std::cout << "		>> RobWork axis: " << test4.axis() << "   |  RobWork angle: " << test4.angle() << std::endl << std::endl;
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;
	
//--------------------------------------------------------------------------------------------------------------

	cout << "Program done. Check genfiles/ to see output file." << endl;
	cout << endl;
	cout << endl;
	cout << endl;
	cout << "				---------------------------------------------------------" << endl;
	cout << "				 Carlos Viescas Huerta | SDU Robotics | RoVi1 - Robotics" << endl;
	cout << "				---------------------------------------------------------" << endl;
	return 0;
}
