/*This is the code for static grasping project
  This code is coded by Fan Yang
  Start from July, 25th 2019
*/
#include <iostream>
#include <string>
#include <signal.h>

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "tasks/PositionTask.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

bool runloop = false;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/hand.urdf";
const string robot_name = "Hand3Finger";


const std::string JOINT_ANGLES_KEY  = "sai2::graspFan::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::graspFan::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEYS = "sai2::graspFan::actuators::fgc";
#define NUM_OF_FINGERS    3

#define PRE_GRASP        0
#define MOVE_CLOSE       1

int state = PRE_GRASP;

int main (int argc, char** argv) 
{
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	double frequency = 1000;

	auto robot = new Sai2Model::Sai2Model(robot_file, false);

		// read from Redis
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();
	int dof = robot->dof();

	VectorXd command_torques = VectorXd::Zero(dof);
	vector<VectorXd> position_command_torques;
	VectorXd palm_command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);
	position_command_torques.push_back(VectorXd::Zero(dof));
	position_command_torques.push_back(VectorXd::Zero(dof));
	position_command_torques.push_back(VectorXd::Zero(dof));
	position_command_torques.push_back(VectorXd::Zero(dof));
	position_command_torques.push_back(VectorXd::Zero(dof));
	//control 4 fingers and the palm, finger 0,1,2,3 and palm


	vector<Sai2Primitives::PositionTask *>  position_tasks;
	vector<string> link_names;
	vector<Affine3d> poses;
	Affine3d identity_pose = Affine3d::Identity();
	Affine3d temp_pose = Affine3d::Identity();
	temp_pose.translation() = Vector3d(0.02,0.0,0.0);
	poses.push_back(temp_pose);
	poses.push_back(identity_pose);
	poses.push_back(identity_pose);
	poses.push_back(identity_pose);
	poses.push_back(identity_pose);

	link_names.push_back("finger0-link3");
	link_names.push_back("finger1-link3");
	link_names.push_back("finger2-link3");
	link_names.push_back("finger3-link3");
	link_names.push_back("palm");
	for (int i = 0; i < NUM_OF_FINGERS; i++)
	{
		position_tasks.push_back(new Sai2Primitives::PositionTask(robot, link_names[i], poses[i], 1/frequency));
	}

	auto palm_posori_task = new Sai2Primitives::PosOriTask(robot, "palm", Vector3d(0.0,0.0,0.0));


	LoopTimer timer;
	timer.setLoopFrequency(frequency);
	timer.setCtrlCHandler(sighandler);
	timer.initializeTimer(1000000);


	runloop = true ;
	//cout << endl << position_tasks[0]._desired_position << endl;
	//cout << endl << position_tasks[1]._desired_position << endl;
	//cout << endl << position_tasks[2]._desired_position << endl;
	while(runloop)
	{

		timer.waitForNextLoop();
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		//cout <<"q" << robot->_q << endl;
		robot->updateModel();
		robot->coriolisForce(coriolis);

		if (state == PRE_GRASP)
		{
			N_prec.setIdentity();
			palm_posori_task->updateTaskModel(N_prec);
			N_prec = palm_posori_task->_N;
			palm_posori_task->_desired_position = Vector3d(0.0,0.0,0.0);
			palm_posori_task->computeTorques(palm_command_torques);
			//cout << "Here's the torque" << palm_command_torques << endl;
			position_tasks[0]->updateTaskModel(N_prec);
			position_tasks[1]->updateTaskModel(N_prec);
			position_tasks[2]->updateTaskModel(N_prec);
			//position_tasks[3]->updateTaskModel(N_prec);
			position_tasks[0]->_desired_position = Vector3d(-0.05, -0.05, -0.05);
			position_tasks[1]->_desired_position = Vector3d(0.08, -0.041, -0.04);
			position_tasks[2]->_desired_position = Vector3d(0.08, 0.0, -0.04);
			position_tasks[0]->computeTorques(position_command_torques[0]);
			position_tasks[0]->computeTorques(position_command_torques[1]);
			position_tasks[0]->computeTorques(position_command_torques[2]);
			cout << endl << palm_command_torques << endl;
			cout << endl << position_command_torques[0] << endl;			
		}
	}
	command_torques = position_command_torques[0] +position_command_torques[1] \
	+ position_command_torques[2] + position_command_torques[3] \
	+ position_command_torques[4] + palm_command_torques;
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS, command_torques);
	command_torques.setZero();
	return 0;
}