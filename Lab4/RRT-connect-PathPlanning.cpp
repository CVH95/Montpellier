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

#define MAXTIME 10.

// Function to print a LUA script template.
void luascript(QPath path, int eps, int index1, int index2, const string deviceName, const string bottle_frame_name, const string tool_frame_name, const string table_frame_name) 
{

	ofstream luafile;
	stringstream ss;
	string LuaFilePath = "/home/student/workspace/Robotics_Projects/mandatory_ex2/LUA/epsilon_";
	string fileMain = "_LUAscript_";
	string experiment = "_";
	string LuaExtension = ".lua";
	ss<<LuaFilePath<<eps<<fileMain<<index1<<experiment<<index2<<LuaExtension;
	string LuaFileName = ss.str();
	luafile.open(LuaFileName);

	luafile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
	luafile << "state = wc:getDefaultState()" << endl; 
	luafile << "device = wc:findDevice(\"" << deviceName << "\")" << endl; 
	luafile << "gripper = wc:findFrame(\"" << tool_frame_name << "\");" << endl; 
	luafile << "bottle = wc:findFrame(\"" << bottle_frame_name << "\");" << endl; 
	luafile << "table = wc:findFrame(\"" << table_frame_name << "\");" << endl; 
	luafile << "   " << endl;
	luafile << "function setQ(q)" << endl;
	luafile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
	luafile << "device:setQ(qq,state)" << endl;
	luafile << "rws.getRobWorkStudio():setState(state)" << endl;
	luafile << "rw.sleep(0.1)" << endl;
	luafile << "end" << endl;
	luafile << "   " << endl;
	luafile << "function attach(obj, tool)" << endl;
	luafile << "rw.gripFrame(obj, tool, state)" << endl;
	luafile << "rws.getRobWorkStudio():setState(state)" << endl;
	luafile << "rw.sleep(0.1)" << endl;
	luafile << "end" << endl;
	luafile << "   " << endl;
	luafile << "   " << endl;
	luafile << "setQ({-3.142, -0.827, -3.002, -3.143, 0.099, -1.573})" << endl;
	luafile << "attach(bottle,gripper)" << endl;
	
	for (QPath::iterator it = path.begin(); it < path.end(); it++)  
	{
		Q currentStep = *it;
		luafile << "setQ({" << currentStep[0] << "," << currentStep[1] << "," << currentStep[2] << "," << currentStep[3] << "," << currentStep[4] << "," << currentStep[5] <<"})" << endl;	
	}	
	
	luafile << "attach(bottle,table)" << endl;
	luafile << "setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})" << endl;

	luafile.close();

}


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

// Function that looks for collisions at a given state (Q).
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

// Test with different epsilon to find RRT solutions with minimum path sizes and search times.
void optimization(PlannerConstraint constraint, QSampler::Ptr sampler, QMetric::Ptr metric , double epsilon, Q from, Q to)
{
	vector<float> mean_calculator_size;
	vector<float> mean_calculator_time;
	vector<float> min_mean, min_time;
	ofstream file1, file2;
	file2.open("/home/student/workspace/Robotics_Projects/mandatory_ex2/genfiles/Optimization_meanVectors.txt");
	file1.open("/home/student/workspace/Robotics_Projects/mandatory_ex2/genfiles/Optimization_matrix.txt");
	file1 << "======================================================================================" << endl;
	file1 << "                 		EPSILON OPTIMIZATION" << endl;
	file1 << "======================================================================================" << endl;
	file1 << "  " << endl;
	file2 << "======================================================================================" << endl;
	file2 << "                 		EPSILON OPTIMIZATION - MEAN VECTORS" << endl;
	file2 << "======================================================================================" << endl;
	file2 << "  " << endl;
	file2 << "  " << endl;
	file1 << " Matrix array contains the path size and search time results for each test." << endl;
	file1 << " They are displayed pairwise: [EPSILON=i]:  testj: (pathsize_j, searchtime_j)." << endl; 
	file1 << "  " << endl;
	file1 << "  " << endl;
	file1 << "  " << endl;
	float pathSize_mat[50][10];
	float searchTime_mat[50][10];
	for(int i=0;  i < 50; i++)
	{		
		QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);
		file1 << "[EPSILON = " << epsilon << "]: ";
		cout << "[EPSILON = " << epsilon << "]: ";
		for(int j=0; j<10; j++)
		{	
			QPath path;
			Timer t;
			t.resetAndResume();
			planner->query(from,to,path,MAXTIME);
			t.pause();
			if (t.getTime() >= MAXTIME) 
			{
				cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
			}

			pathSize_mat[i][j] = path.size();
			searchTime_mat[i][j] = t.getTime();
			cout << "test" << j << ":(" << pathSize_mat[i][j] << ", " << searchTime_mat[i][j] << ") ";
			file1 << "test" << j << ":(" << pathSize_mat[i][j] << ", " << searchTime_mat[i][j] << ")  ";
			mean_calculator_size.push_back(path.size());
			mean_calculator_time.push_back(t.getTime());
			
		}
		float size_avg = accumulate(mean_calculator_size.begin(), mean_calculator_size.end(), 0.0)/mean_calculator_size.size();
		float time_avg = accumulate(mean_calculator_time.begin(), mean_calculator_time.end(), 0.0)/mean_calculator_time.size();
		min_mean.push_back(size_avg);
		min_time.push_back(time_avg);
		file1 << endl;
		file1 << endl;
		cout << endl;
		cout << "       MEAN VECTOR: (" << size_avg << ", " << time_avg << ")" << endl;
		cout << endl;
		file2 << "[EPSILON = " << epsilon << "]:  " << "(" << size_avg << ", " << time_avg << ")" << endl;
		epsilon = epsilon + 0.05;
		mean_calculator_size.clear();
		mean_calculator_time.clear();
	}
	
	int minPath = 0, minTime = 0;
    	for (unsigned a = 0; a < min_mean.size(); a++)
    	{
        	if (min_mean[a] < min_mean[minPath]) 
            		minPath = a;
        	if (min_time[a] < min_time[minTime]) 
           		 minTime = a;
    	}
    	cout << "Minimum mean path size is " << min_mean[minPath] << " found at an EPSILON = " << minPath*0.05 + 0.1 << endl;
	file2 << "Minimum mean path size is " << min_mean[minPath] << " found at an EPSILON = " << minPath*0.05 + 0.1 << endl;
    	cout << "Minimum average search time is " << min_time[minTime] << " found at an EPSILON = " << minTime*0.05 + 0.1 << endl;
	file2 << "Minimum average search time is " << min_time[minTime] << " found at an EPSILON = " << minTime*0.05 + 0.1 << endl;
	file1.close();
	file2.close();
}

// RRT - connect path finder with a fixed epsilon
void RepeatedRRT(PlannerConstraint constraint, QSampler::Ptr sampler, QMetric::Ptr metric , double epsilon, Q from, Q to, int index1)
{

	const string deviceName = "KukaKr16";
	const string bottle_frame_name = "Bottle";
	const string tool_frame_name = "Tool";
	const string table_frame_name = "Table";
	
	int eps = epsilon*100;

	stringstream ss;
	string FilePath = "/home/student/workspace/Robotics_Projects/mandatory_ex2/genfiles/epsilon_";
	string fileMain = "_RepeatedRRT";
	string Extension = ".txt";
	ss<<FilePath<<eps<<fileMain<<Extension;
	string FileName = ss.str();

	ofstream file2;
	file2.open(FileName);
	file2 << "======================================================================================" << endl;
	file2 << "                 		PATH GENERATOR" << endl;
	file2 << "======================================================================================" << endl;
	file2 << " EPSILON = " << epsilon << endl;
	file2 << "  " << endl;
	file2 << "  " << endl;

	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);
	
	vector<float> path_accumulator;

	int LUAfile_index2 = 1;
	
	for(int j=0; j<30; j++)
	{	
		QPath path;
		Timer t;
		t.resetAndResume();
		planner->query(from,to,path,MAXTIME);
		t.pause();
		if (t.getTime() >= MAXTIME) 
		{
			cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
		}
		
		cout << "PATH #" << j << ":" << endl;
		cout << "Length = " << path.size() << ". Found in " << t.getTime() << " seconds." << endl;
		cout << " " << endl;
		file2 << "PATH #" << j << ":" << endl;
		file2 << "Length = " << path.size() << ". Found in " << t.getTime() << " seconds." << endl;
		file2 << " " << endl;
		int i = 0;
		for (QPath::iterator it = path.begin(); it < path.end(); it++) 
		{
			cout << "RRT node " << i << ": {" << *it << "}" << endl;
			file2 << "RRT node " << i << ": {" << *it << "}" << endl;
			i = i+1;
	
		}
		// WRITING LUA SCRIPT	
		luascript(path, eps, index1, LUAfile_index2, deviceName, bottle_frame_name, tool_frame_name, table_frame_name);
		
		path_accumulator.push_back(path.size());
		LUAfile_index2 = LUAfile_index2 +1;
		file2 << endl;
		file2 << endl;
		cout << endl;
		cout << endl;
		
	}
	cout << "Configurations at epsilon" << epsilon << "found with the following path sizes:" << endl; 
	file2 << "Configurations at epsilon" << epsilon << "found with the following path sizes:" << endl; 
	for(unsigned k=0; k < path_accumulator.size(); k++)
	{
		cout << path_accumulator[k] << " ";
		file2 << path_accumulator[k] << " ";
	}
	cout << endl;
	file2.close();
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	rw::math::Math::seed();
	
	// LOADING WORKCELL AND GENERAL INFO

	cout << endl;	
	cout << "PATH PLANNING WITH KUKA KR16" << endl;
	cout << "Carlos Viescas Huerta, Matheswaran Pitchai." << endl;
	// Looking for workcell
	const string wcFile = "/home/student/workspace/Robotics_Projects/mandatory_ex2/Kr16WallWorkCell/Scene.wc.xml"; 
	const string deviceName = "KukaKr16";
	const string bottle_frame_name = "Bottle";
	const string tool_frame_name = "Tool";
	const string table_frame_name = "Table";
	
	cout << "WorkCell " << wcFile << " and device " << deviceName << endl;

	// if found, loading workcell
	WorkCell::Ptr wc = WorkCellFactory::load(wcFile);	
	// and device
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}

	cout << "WorkCell contains the following devices:" << endl;
	cout << "  " << endl;
	BOOST_FOREACH(Device::Ptr dvcs, wc->getDevices()){
		cout << ">>  " << dvcs->getName() << endl;
	}
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;
	cout << "KINEMATIC TREE:" << endl;
	// Display kinematic tree for given workcell to see all frame names.
	printKinTree(*wc->getWorldFrame(), wc->getDefaultState(), Transform3D<>::identity(), 0);

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// INITIAL SETTINGS
	
	// Finding frames of the bottle and the tool.
	rw::kinematics::Frame* bottle_frame =  wc->findFrame("Bottle");
	rw::kinematics::Frame* tool_frame =  wc->findFrame("Tool");
		
	Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	// Initiate state variable by default.
	State state = wc->getDefaultState(); 
	// Set new state to q_pick position.
	device->setQ(from,state);	
	// Grasping the bottle.
	rw::kinematics::Kinematics::gripFrame(bottle_frame, tool_frame, state);	
	// Set collision detection strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 
	// Set the planner constraint to build RRT-connect.
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state); 

	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	// Set initial epsilon
	double epsilon = 0.1;
	// Final position q_place
	Q to(6,1.571,0.006,0.030,0.153,0.762,4.490);
	// Check for collisions at initial configuration.
	if (!checkCollisions(device, state, detector, from))
		return 0;
	// Check for collisions at final configuration.
	if (!checkCollisions(device, state, detector, to))
		return 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// OPTIMIZATION

	cout << "Displaying matrix of vector pairs (pathSize, searchTime) for each value of EPSILON." << endl;
	cout << "  " << endl;	
	// Call to optimization function -> Looking for epsilon that return mimimum path size and minimum search time.
	optimization(constraint, sampler, metric , epsilon, from, to);
	
	int continue1;
	cout << "Phase 1 completed. Do you want to continue? (yes=1/no=0)  ";
	cin >> continue1;
	if(continue1 == 0) {return 0;}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// RRT PATH FINDING

	double EPS_minP, EPS_minT, EPS1;
	cout << "Select epsilon value that makes path.size() minimum:  ";
	cin >>EPS_minP;
	cout << "Select epsilon value that makes search time minimum:  ";
	cin >> EPS_minT;
	cout << "Select other epsilon value:  ";
	cin >> EPS1;
	
	int exp1 = 1;
	int exp2 = 2;
	int exp3 = 3;
	// Path solving for the different epsilons.
	RepeatedRRT(constraint, sampler, metric , EPS_minP, from, to, exp1);
	RepeatedRRT(constraint, sampler, metric , EPS_minT, from, to, exp2);
	RepeatedRRT(constraint, sampler, metric , EPS1, from, to, exp3);
	

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


	cout << "Program done. Data was saved succesfully to file in ~/workspace/Robotics_Projects/mandatory_ex2/genfiles" << endl;
	return 0;
}
