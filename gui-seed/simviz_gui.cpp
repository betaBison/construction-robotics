#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "uiforce/UIForceWidget.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface
#include "uiforce/UIForceWidget.h"
#include <iostream>
#include <string>
#include <thread>
#include <cmath>
#include <csignal>
#include <signal.h>

#include "keys_gui.h"

using namespace Eigen;

// redis keys
constexpr const char *world_file = "resources/world.urdf";
constexpr const char *robot_name = "mmp_panda";
constexpr const char *camera_name = "camera_fixed";
constexpr const char *SIM_TITLE = "SAI2.0 Example - Mobile Panda";

RedisClient redis_client;
bool fSimulationRunning = false;

void sighandler(int)
{
	fSimulationRunning = false;
}

// simulation and control loop
void simulation(Sai2Model::Sai2Model *robot, Simulation::Sai2Simulation *sim,
				UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char *description);

// callback to print glew errors
bool glewInitialize();

// initialize window manager
GLFWwindow *glfwInitialize();



// callback when a key is pressed
void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow *window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotCircle = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main(int argc, char **argv)
{
	std::cout << "Loading URDF world model file: " << world_file << std::endl;

	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	auto robot = new Sai2Model::Sai2Model(ROBOT_FILE, false, sim->getRobotBaseTransform(robot_name), world_gravity);

	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow *window = glfwInitialize();

	double last_cursorx, last_cursory;

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// init click force widget
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	// initialize glew
	glewInitialize();

	// start the simulation thread first
	fSimulationRunning = true;
	std::thread sim_thread(simulation, robot, sim, ui_force_widget);


	const string control_link = "linkTool";
	const Vector3d control_point = Vector3d(0,0.104,0.203);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	#ifdef USING_OTG
		posori_task->_use_interpolation_flag = true;
	#else
		posori_task->_use_velocity_saturation_flag = true;
	#endif


	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp)
		{
			camera_pos = camera_pos + 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat + 0.05 * cam_roll_axis;
		}
		if (fTransXn)
		{
			camera_pos = camera_pos - 0.05 * cam_roll_axis;
			camera_lookat = camera_lookat - 0.05 * cam_roll_axis;
		}
		if (fTransYp)
		{
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05 * cam_up_axis;
			camera_lookat = camera_lookat + 0.05 * cam_up_axis;
		}
		if (fTransYn)
		{
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05 * cam_up_axis;
			camera_lookat = camera_lookat - 0.05 * cam_up_axis;
		}
		if (fTransZp)
		{
			camera_pos = camera_pos + 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat + 0.1 * cam_depth_axis;
		}
		if (fTransZn)
		{
			camera_pos = camera_pos - 0.1 * cam_depth_axis;
			camera_lookat = camera_lookat - 0.1 * cam_depth_axis;
		}
		if (fRotCircle) {
			// look at the robot base
			float diff_x = camera_pos(0) - robot->_q(0);
			float diff_y = camera_pos(1) - robot->_q(1);
			float r = sqrt(pow(diff_x,2) + pow(diff_y,2));
			float theta = atan2(diff_y, diff_x);
			theta -= 0.01;
			// rotate the angle slightly
			camera_pos(0) = robot->_q(0) + r*cos(theta);
			camera_pos(1) = robot->_q(1) + r*sin(theta);

			camera_lookat(0) = robot->_q(0);
			camera_lookat(1) = robot->_q(1);
			camera_lookat(2) = robot->_q(3);


			// look at end effector
			// sim->getJointPositions(robot_name, robot->_q);
			// float diff_x = camera_pos(0) - posori_task->_current_position(0);
			// float diff_y = camera_pos(1) - posori_task->_current_position(1);
			// float r = sqrt(pow(diff_x,2) + pow(diff_y,2));
			// float theta = atan2(diff_y, diff_x);
			// theta -= 0.01;
			// // rotate the angle slightly
			// camera_pos(0) = posori_task->_current_position(0) + r*cos(theta);
			// camera_pos(1) = posori_task->_current_position(1) + r*sin(theta);


			camera_lookat = posori_task->_current_position;
			//cout << "pose: " << camera_pos << "\n look: " << camera_lookat << "\n\n\n";

		}
		if (fRotPanTilt)
		{
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006 * (cursorx - last_cursorx);
			double azimuth = 0.006 * (cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt;
			m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt * (camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan;
			m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan * (camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();
	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model *robot, Simulation::Sai2Simulation *sim, UIForceWidget *ui_force_widget)
{

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(SIM_JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// init variables
	VectorXd g(dof);

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	fSimulationRunning = true;
	while (fSimulationRunning)
	{
		fTimerDidSleep = timer.waitForNextLoop();

		// read command torques from redis and apply to simulation
		command_torques = redis_client.getEigenMatrixJSON(SIM_JOINT_TORQUES_COMMANDED_KEY);
		sim->setJointTorques(robot_name, command_torques);

		ui_force_widget->getUIForce(ui_force);
		ui_force_widget->getUIJointTorques(ui_force_command_torques);

		if (fRobotLinkSelect)
			sim->setJointTorques(robot_name, command_torques + ui_force_command_torques);
		else
			// get gravity torques
			robot->gravityVector(g);
			sim->setJointTorques(robot_name, command_torques + g);

		// integrate forward
		sim->integrate(0.001);

		// read robot state and update redis
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();

		redis_client.setEigenMatrixJSON(SIM_JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixJSON(SIM_JOINT_VELOCITIES_KEY, robot->_dq);
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles() / end_time << "Hz\n";
}

//------------------------------------------------------------------------------
GLFWwindow *glfwInitialize()
{
	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor *primary = glfwGetPrimaryMonitor();
	const GLFWvidmode *mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow *window = glfwCreateWindow(windowW, windowH, SIM_TITLE, nullptr, nullptr);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char *description)
{
	std::cerr << "GLFW Error: " << description << std::endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// exit application
		glfwSetWindowShouldClose(window, GL_TRUE);
		break;
	case GLFW_KEY_RIGHT:
		fTransXp = set;
		break;
	case GLFW_KEY_LEFT:
		fTransXn = set;
		break;
	case GLFW_KEY_UP:
		fTransYp = set;
		break;
	case GLFW_KEY_DOWN:
		fTransYn = set;
		break;
	case GLFW_KEY_A:
		fTransZp = set;
		break;
	case GLFW_KEY_Z:
		fTransZn = set;
		break;
	case GLFW_KEY_R:
		fRotCircle = set;
		break;
	default:
		break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow *window, int button, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button)
	{
	// left click pans and tilts
	case GLFW_MOUSE_BUTTON_LEFT:
		fRotPanTilt = set;
		// NOTE: the code below is recommended but doesn't work well
		// if (fRotPanTilt) {
		// 	// lock cursor
		// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		// } else {
		// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		// }
		break;
	// if right click: don't handle. this is for menu selection
	case GLFW_MOUSE_BUTTON_RIGHT:
		fRobotLinkSelect = set;
		break;
	// if middle click: don't handle. doesn't work well on laptops
	case GLFW_MOUSE_BUTTON_MIDDLE:
		break;
	default:
		break;
	}
}