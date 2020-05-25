// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1
#define BASE_NAV              2
#define BASE_ELEV             3
#define A_SIDE_BOTTTOM        4
#define A_SIDE_TOP            5
#define B_SIDE_BOTTOM         6
#define B_SIDE_TOP            7

int control_type = JOINT_CONTROLLER;
int state = BASE_NAV;


// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::robot::mmp_panda::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::robot::mmp_panda::actuators::fgc";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "linkTool";
	const Vector3d control_point = Vector3d(0.203,0,0.104);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	#ifdef USING_OTG
		posori_task->_use_interpolation_flag = true;
	#else
		posori_task->_use_velocity_saturation_flag = true;
	#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	#ifdef USING_OTG
		joint_task->_use_interpolation_flag = true;
	#else
		joint_task->_use_velocity_saturation_flag = true;
	#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	q_init_desired << 0.41, 1.76, 1.12, -.07, -.13, .14, .49, 5.6, 1.36, 2, 7.27; //0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// 0.41, 1.76, 1.12, -.07, -.13, .14, .49, 5.6, 1.36, 2, 7.27
	//q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		// if(state == BASE_NAV){

		// }

		// else if(state == BASE_ELEV){

		// }

		// else if(state == A_SIDE_BOTTTOM){

		// }

		// else if(state == A_SIDE_TOP){

		// }

		// else if(state == B_SIDE_BOTTOM){

		// }

		// else if(state == B_SIDE_TOP){

		// }
	
		if(control_type == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_init_desired).norm() < 0.15 )
			{
				posori_task->reInitializeTask();
				posori_task->_desired_position = Vector3d(0,2.18,2.28); //posori_task->_desired_position += Vector3d(0,2.18,2.28)
				posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;

				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				control_type = POSORI_CONTROLLER;
			}
		}

		else if(control_type == POSORI_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
