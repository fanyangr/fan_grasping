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
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::graspFan::actuators::fgc";

#define NUM_OF_FINGERS_IN_MODEL 4
#define NUM_OF_FINGERS_USED     3

#define PRE_GRASP               0
#define FINGER_MOVE_CLOSE       1
#define CHECK                   2

int state = PRE_GRASP;

double prob_distance = 0.002;

// the function used in the finger position control command
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position);
// the function used in the finger force control, used to achieve compliance
VectorXd compute_force_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position);
// this function is used to detect surface normal by sampling several points in the vicinity
// It can only be called when the finger tip is making contact with the object surface
// returns the torque needed 
VectorXd detect_surface_normal(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d original_pos, Vector3d CoM_of_object, int& state, Vector3d& normal, deque<double>& velocity_record);


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

	vector<Vector3d> current_finger_position;
	VectorXd command_torques = VectorXd::Zero(dof);
	vector<VectorXd> finger_command_torques;
	VectorXd palm_command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);
	finger_command_torques.push_back(VectorXd::Zero(dof));
	finger_command_torques.push_back(VectorXd::Zero(dof));
	finger_command_torques.push_back(VectorXd::Zero(dof));
	finger_command_torques.push_back(VectorXd::Zero(dof));
	vector<VectorXd> temp_finger_command_torques = finger_command_torques; // the raw command torques before blocking
	//control 4 fingers, finger 0,1,2,3

	// the vector used to record the velocity in the finger move close state
	vector<deque<double>> velocity_record;
	vector<deque<double>> detect_velocity_record;	


	// vector<Sai2Primitives::PositionTask *>  position_tasks;
	vector<int> detect_states;
	vector<Vector3d> normals;
	vector<string> link_names;
	vector<Affine3d> poses;
	Affine3d identity_pose = Affine3d::Identity();
	Affine3d temp_pose = Affine3d::Identity();
	temp_pose.translation() = Vector3d(0.0507,0.0,0.0);
	poses.push_back(temp_pose);
	temp_pose.translation() = Vector3d(0.0327, 0.0, 0.0);
	poses.push_back(temp_pose);
	poses.push_back(temp_pose);
	poses.push_back(temp_pose);

	link_names.push_back("finger0-link3");
	link_names.push_back("finger1-link3");
	link_names.push_back("finger2-link3");
	link_names.push_back("finger3-link3");

	Vector3d CoM_of_object = Vector3d(0.0,0.0,0.05); // in the world frame
	CoM_of_object -= Vector3d(0.0, 0.0, 0.25); // transform into the robor frame

	for(int i = 0; i < NUM_OF_FINGERS_USED; i++)
	{
		deque<double> temp_queue;
		temp_queue.push_back(0.0);
		temp_queue.push_back(0.0);
		velocity_record.push_back(temp_queue);
		detect_velocity_record.push_back(temp_queue);

		current_finger_position.push_back(Vector3d::Zero());

		detect_states.push_back(0);

		normals.push_back(Vector3d::Zero());
	}

	auto palm_posori_task = new Sai2Primitives::PosOriTask(robot, "palm", Vector3d(0.0,0.0,0.0));

	LoopTimer timer;
	timer.setLoopFrequency(frequency);
	timer.setCtrlCHandler(sighandler);
	//timer.initializeTimer(1000000);

	vector<int> finger_contact_flag; // finger0, 1, 2, 3
	for (int i = 0; i < NUM_OF_FINGERS_IN_MODEL; i++)
	{
		finger_contact_flag.push_back(0);
	}

	runloop = true ;
	int loop_counter = 0;


	// cout << robot->_joint_names_map["finger0-j0"] << "!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
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
			palm_posori_task->_desired_position = Vector3d(0.03,0.0,-0.08);
			N_prec.setIdentity();
			palm_posori_task->updateTaskModel(N_prec);
			N_prec = palm_posori_task->_N;
			palm_posori_task->computeTorques(palm_command_torques);
			//cout << "Here's the torque" << palm_command_torques << endl;
			temp_finger_command_torques[0] = compute_position_cmd_torques(robot, link_names[0], poses[0].translation(), Vector3d(-0.08, 0.0, -0.19));
			temp_finger_command_torques[1] = compute_position_cmd_torques(robot, link_names[1], poses[1].translation(), Vector3d(0.15, -0.041, -0.19));
    		temp_finger_command_torques[2] = compute_position_cmd_torques(robot, link_names[2], poses[2].translation(), Vector3d(0.15, 0.0, -0.19));
    		temp_finger_command_torques[3] = compute_position_cmd_torques(robot, link_names[3], poses[3].translation(), Vector3d(0.15, 0.041, -0.09));
		    		
    		// block the unrelated torques
    		finger_command_torques[0].block(6,0,4,1) = temp_finger_command_torques[0].block(6,0,4,1);
    		finger_command_torques[1].block(10,0,4,1) = temp_finger_command_torques[1].block(10,0,4,1);
    		finger_command_torques[2].block(14,0,4,1) = temp_finger_command_torques[2].block(14,0,4,1);
    		finger_command_torques[3].block(18,0,4,1) = temp_finger_command_torques[3].block(18,0,4,1);



    		if (palm_command_torques.norm() + finger_command_torques[0].norm() + finger_command_torques[1].norm() + finger_command_torques[2].norm() < 0.00003)
    		{
    			state = FINGER_MOVE_CLOSE;
    		}
		}
		else if (state == FINGER_MOVE_CLOSE)
		{	
			// keep the position of the palm
			palm_posori_task->_desired_position = Vector3d(0.03,0.0,-0.08);
			N_prec.setIdentity();
			palm_posori_task->updateTaskModel(N_prec);
			N_prec = palm_posori_task->_N;
			palm_posori_task->computeTorques(palm_command_torques);

			// force controller for the fingers
			for(int i = 0; i < NUM_OF_FINGERS_USED; i++)
			{
				if (finger_contact_flag[i] == 0)
				{
					temp_finger_command_torques[i] = compute_force_cmd_torques(robot, link_names[i], poses[i].translation(), CoM_of_object);
					finger_command_torques[i].block(6+4*i,0,4,1) = temp_finger_command_torques[i].block(6+4*i,0,4,1);
					Vector3d temp_finger_velocity = Vector3d::Zero();
					robot->linearVelocity(temp_finger_velocity, link_names[i], poses[i].translation());
					velocity_record[i].pop_front();
					velocity_record[i].push_back(temp_finger_velocity.norm());
					if (velocity_record[i][1]/velocity_record[i][0] < 0.5)
					{
						cout <<"finger "<< i <<" contact"<<endl;
						finger_contact_flag[i] = 1;
						// set the desired position, maintain the current position
						robot->position(current_finger_position[i], link_names[i], poses[i].translation());
					}
				}
				// maintain the current position after contact
				else if (finger_contact_flag[i] == 1)
				{
					temp_finger_command_torques[i] = compute_position_cmd_torques(robot, link_names[i], poses[i].translation(), current_finger_position[i]);
    				finger_command_torques[i].block(6 + 4 * i ,0 ,4, 1) = temp_finger_command_torques[i].block(6 + 4 * i, 0 ,4 ,1 );
				} 
			}

			int sum_of_contact = 0;
			for (int j = 0; j < NUM_OF_FINGERS_USED; j++)
			{
				sum_of_contact += finger_contact_flag[j];
			}
			if (sum_of_contact == 3)
			{
				state = CHECK;
			}
		}

		else if (state == CHECK)
		{
			palm_posori_task->_desired_position = Vector3d(0.03,0.0,-0.08);
			N_prec.setIdentity();
			palm_posori_task->updateTaskModel(N_prec);
			N_prec = palm_posori_task->_N;
			palm_posori_task->computeTorques(palm_command_torques);
			for (int i = 0; i < NUM_OF_FINGERS_USED; i++)
			{
				temp_finger_command_torques[i] = detect_surface_normal(robot, link_names[i], poses[i].translation(), current_finger_position[i], CoM_of_object, detect_states[i], normals[i], detect_velocity_record[i]);
 				finger_command_torques[i].block(6 + 4 * i ,0 ,4, 1) = temp_finger_command_torques[i].block(6 + 4 * i, 0 ,4 ,1 );
				if (i == 0)
				{
					cout << finger_command_torques[i].block(6 + 4 * i ,0 ,4, 1) << endl;
				}
			}

		}

		loop_counter++;

	command_torques = finger_command_torques[0] + finger_command_torques[1] \
	+ finger_command_torques[2] + finger_command_torques[3]\
	+ palm_command_torques + coriolis;
	//cout << command_torques << endl;
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		// reset to 0
	for(int i =0; i < NUM_OF_FINGERS_USED; i++)
	{
		temp_finger_command_torques[i].setZero();
		finger_command_torques[i].setZero();
	}
	command_torques.setZero();
	}

	command_torques.setZero();
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position)
{
	double kp = 10;
	double kv = 2;
	double damping = -0.0001;

	int dof = robot->dof();
	Vector3d current_position; // in robot frame
	Vector3d current_velocity;
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	robot->Jv(Jv, link, pos_in_link);
	robot->position(current_position, link, pos_in_link);
	robot->linearVelocity(current_velocity, link, pos_in_link);
	VectorXd torque = VectorXd::Zero(dof);
	torque = Jv.transpose()*(kp*(desired_position - current_position) - kv * current_velocity) + damping * robot->_dq;
	return torque;
}

VectorXd compute_force_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position)
{
	int dof = robot->dof();
	double force_requeired = 0.001;
	double damping = -0.0001;
	Vector3d current_position; // in robot frame
	Vector3d current_velocity;
	Vector3d desired_force;
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	robot->Jv(Jv, link, pos_in_link);
	robot->position(current_position, link, pos_in_link);
	desired_force = desired_position - current_position;
	desired_force = desired_force / desired_force.norm(); // normalization
	desired_force = desired_force * force_requeired;
	VectorXd torque = VectorXd::Zero(dof);
	torque = Jv.transpose()*desired_force + damping * robot->_dq;
	return torque;
}

VectorXd detect_surface_normal(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d original_pos, Vector3d CoM_of_object, int& state, Vector3d& normal, deque<double>& velocity_record)
{
	int dof = robot->dof();
	VectorXd torque = VectorXd::Zero(dof);
	Vector3d current_position = Vector3d::Zero();
	robot->position(current_position, link, pos_in_link);
	if(state == 0) // just start from the initial centroid position
	{

		Vector3d desired_position = 0.01*(original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm() + \
		original_pos + Vector3d(0.0, 0.0, prob_distance);
		torque = compute_position_cmd_torques(robot, link, pos_in_link, desired_position);
		if((desired_position - current_position).norm() < 0.001)
		{
			state = 1;
			cout << "!!!!!! " <<link << endl;
		}
	}
	else if (state == 1) // has reached the first intermediate point
	{
		torque = compute_force_cmd_torques(robot, link, pos_in_link, CoM_of_object);
		Vector3d temp_finger_velocity = Vector3d::Zero();
		robot->linearVelocity(temp_finger_velocity, link, pos_in_link);
		velocity_record.pop_front();
		velocity_record.push_back(temp_finger_velocity.norm());
		if (velocity_record[1]/velocity_record[0] < 0.5)
		{
			state = 2;
			cout << link <<" contact"<<endl;
		}
	}

	return torque;

}