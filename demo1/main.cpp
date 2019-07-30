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

const string hand_file = "resources/hand.urdf"

const vector<string> finger_files =
{ "resources/finger0.urdf"} ;
const int n_finger = finger_files.size();
const string hand_name = "Hand3Finger";
const vector<string> finger_names = {"Finger0"} ;

const string HAND_JOINT_TORQUES_COMMAND_KEY = "sai2::graspFan::hand::command_torques";
const vector<std::string> FINGER_JOINT_TORQUES_COMMANDED_KEYs = 
{"sai2::graspFan::finger0::command_torques"};
// - write:
const string HAND_JOINT_ANGLES_KEY = "sai2::graspFan::hand3finger::q";
const string HADN_JOINT_VELOCITIES_KEY = "sai2::graspFan::hand3finger::dq";
const vector<std::string> FINGER_JOINT_ANGLES_KEYs  = {"sai2::graspFan::finger0::q"};
const vector<std::string> FINGER_JOINT_VELOCITIES_KEYs = {"sai2::graspFan::finger0::dq"};
const std::string SIM_TIMESTAMP_KEYs = "sai2::graspFan::simulation::timestamp";

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

	vector<Sai2Model::Sai2Model*> fingers;
	vector<int> finger_dof;

	auto hand = new Sai2Model::Sai2Model(hand_file, false);

	hand->_q = redis_client.getEigenMatrixJSON(HAND_JOINT_ANGLES_KEY);
	hand->_dq = redis_client.getEigenMatrixJSON(HAND_JOINT_VELOCITIES_KEY);
	hand->updateModel();
	int hand_dof = hand->dof();
	for (int i=0; i < n_finger; i++)
	{
		fingers.push_back(new Sai2Model::Sai2Model(finger_files[i],false));
		fingers[i]->_q = redis_client.getEigenMatrixJSON(FINGER_JOINT_ANGLES_KEYs[i]);
		fingers[i]->_dq = redis_client.getEigenMatrixJSON(FINGER_JOINT_VELOCITIES_KEYs[i]);
		fingers[i]->updateModel();
		finger_dof.push_back(fingers[i]->dof());
	}

	// initialize command_torques
	VectorXd hand_command_torques = VectorXd::Zero(hand_dof);

	vector<VectorXd> finger_command_torques;
	for (int i = 0; i < n_finger; i++)
	{
		command_torques.push_back(VectorXd::Zero(finger_dof[i]));
	}

	vector<VectorXd> finger_position_command_torques;
	vector<VectorXd> finger_jonit_command_toruqes;
	for (int i = 0; i < n_finger; i++)
	{
		finger_position_command_torques.push_back(VectorXd::Zero(finger_dof[i]));
		finger_jonit_command_toruqes.push_back(VectorXd::Zero(finger_dof[i]));
	} 

	// initialize the tasks
	vector<Sai2Primitives::PositionTask *>  finger_position_tasks;
	vector<Sai2Primitives::JointTask *> finger_joint_tasks;
	vector<VectorXd> desired_positions;
	vector<string> link_names;
	vector<Affine3d> poses;
	Affine3d identity_pose = Affine3d::Identity();
	Affine3d temp_pose = Affine3d::Identity();
	temp_pose.translation() = Vector3d(0.02,0.0,0.0);
	poses.push_back(temp_pose);
	poses.push_back(identity_pose);
	poses.push_back(identity_pose);
	poses.push_back(identity_pose);
	
	link_names.push_back("finger0-link3");
	link_names.push_back("finger1-link3");
	link_names.push_back("finger2-link3");
	link_names.push_back("finger3-link3");
	

	for (int i = 0; i < n_robot - 1; i++)
	{
		finger_position_tasks.push_back(new Sai2Primitives::PositionTask(fingers[i], link_names[i], poses[i]));
		finger_joint_tasks.push_back(new Sai2Primitives::JointTask(robot));
	}
	auto palm_posori_task = new Sai2Primitives::PosOriTask(robot, "palm", Vector3d(0.0,0.0,0.0));
	//auto test_task =  new Sai2Primitives::PositionTask(robot, "palm", Vector3d(0.0,0.0,0.0));

	LoopTimer timer;
	timer.setLoopFrequency(frequency);
	timer.setCtrlCHandler(sighandler);
	timer.initializeTimer(1000000);


	runloop = true ;
	//cout << endl << position_tasks[0]._desired_position << endl;
	//cout << endl << position_tasks[1]._desired_position << endl;
	//cout << endl << position_tasks[2]._desired_position << endl;
	int loop_counter = 0;
	while(runloop)
	{

		timer.waitForNextLoop();
		for (int i=0;i < n_robot; i++)
		{
			robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
			robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
			//cout <<"q" << robot->_q << endl;
			robots[i]->updateModel();
		}

		// robots[0]->coriolisForce(coriolis);// ????????? how to deal with coriolis

		if (state == PRE_GRASP)
		{
			//the desired position for finger 0 1,2, it's only used for the position task of fingers
			desired_position.push_back(Vector3d(-0.05, -0.05, -0.05));
			desired_position.push_back(Vector3d(0.08, -0.041, -0.04));
			desired_position.push_back(Vector3d(0.08, 0.0, -0.04));

			palm_posori_task->_desired_position = Vector3d(0.0,0.0,0.0);
			// N_prec.setIdentity();
			// palm_posori_task->updateTaskModel(N_prec);
			// N_prec = palm_posori_task->_N;
			//cout << N_prec << endl << endl;
			//test_task->_desired_position = Vector3d(1.0,1.0,0.0);
			//test_task->_kp=100;
			//test_task->_kv = 100;
			//test_task->_ki = 100;

			palm_posori_task->computeTorques(command_torques[0]);
			//test_task->computeTorques(palm_command_torques);
			//cout << "Here's the torque" << palm_command_torques << endl;

			//test_position_task->_desired_position = Vector3d(0.08, -0.041, -0.04);
			N_prec.setIdentity();
			for (int i = 0; i < n_robot-1; i++)
			{
				finger_position_tasks[i]->_desired_position = desired_position[i];
				finger_position_tasks[i]->updateTaskModel(N_prec);
				finger_joint_tasks[i]->updateTaskModel(finger_position_tasks[i]->_N);
			}
			
			//position_tasks[0]->updateTaskModel(N_prec);
			//position_tasks[1]->updateTaskModel(N_prec);
			//position_tasks[2]->updateTaskModel(N_prec);
			//position_tasks[3]->updateTaskModel(N_prec);
			test_position_task->computeTorques(test_command_torques);

			// cout << test_command_torques <<endl;

			position_tasks[0]->computeTorques(position_command_torques[0]);
			position_tasks[1]->computeTorques(position_command_torques[1]);
			position_tasks[2]->computeTorques(position_command_torques[2]);
			//cout << endl << palm_command_torques << endl;
			// cout << endl << position_command_torques[1] << endl;
			// if (loop_counter >= 1 )
			// {
			// 	break;
			// }
			loop_counter++;			
		}

	command_torques = position_command_torques[0] +position_command_torques[1] \
	+ position_command_torques[2] + position_command_torques[3]\
	+ palm_command_torques + coriolis;
		// command_torques = palm_command_torques + test_command_torques + coriolis;
	//cout << command_torques << endl;
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[0], command_torques);
	command_torques.setZero();
	}

	command_torques.setZero();
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEYS[0], command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}