#include "Drone.h"
#include <iomanip>

Drone::Drone()
	: physics_(1.0),
	altitude_pid_(5.0, 1.0, 0.5, 10.0),
	roll_pid_(4.0, 0.2, 0.1, 1.0),
	pitch_pid_(4.0, 0.2, 0.1, 1.0),
	yaw_pid_(1.0, 0.01, 0.05, 0.5),
	target_altitude_(0.0),
	target_pitch_(0.0), target_roll_(0.0), target_yaw_(0.0)
{
	reset();
}

void Drone::reset()
{
	state_ = { 0 }; // zero initialize
	state_.z = 0.0;
	state_.x = state_.y = 0.0;
	state_.vx = state_.vy = state_.vz = 0.0;
	state_.roll = state_.pitch = state_.yaw = 0.0;
	roll_pid_.reset();
	pitch_pid_.reset();
	yaw_pid_.reset();
	altitude_pid_.reset();
}

void Drone::setMass(double mass)
{
	physics_ = PhysicsEngine(mass);
}

void Drone::setTargetAltitude(double altitude)
{
	target_altitude_ = altitude;
}

void Drone::setTargetAngles(double roll, double pitch, double yaw)
{
	target_roll_ = roll;
	target_pitch_ = pitch;
	target_yaw_ = yaw;
}

const State& Drone::state() const
{
	return state_;
}

void Drone::enableLogging(const std::string& filename)
{
	log_file_.open(filename);
	if (log_file_.is_open()) {
		log_file_ << std::fixed << std::setprecision(4);
		log_file_ << "Time,X,Y,Z,VX,VY,VZ,Roll,Pitch,Yaw,RollRate,PitchRate,YawRate\n";
	}
}

void Drone::update(double dt)
{
	// Altitude control
	double altitude_error = target_altitude_ - state_.z;
	double thrust_command = altitude_pid_.update(altitude_error, dt); // this returns thrust acceleration targett (m/s^2)
	// Convert desired acceleration to thrust force: F = m * (a + g)
	double thrust_force = physics_.mass() * (thrust_command + 9.81); //mass is private but for demonstration keep it public? We'll access via friend or change; easiest is to expose mass: make mass public in PhysicsEngine or multiply by 1.0 mass
	if (thrust_force < 0.0) {
		thrust_force = 0.0; // no negative thrust
	}
	if (thrust_force > 30.0) {
		thrust_force = 30.0; // max thrust limit
	}
	// Attitude control
	double roll_error = target_roll_ - state_.roll;
	double pitch_error = target_pitch_ - state_.pitch;
	double yaw_error = target_yaw_ - state_.yaw;
	double roll_torque = roll_pid_.update(roll_error, dt);
	double pitch_torque = pitch_pid_.update(pitch_error, dt);
	double yaw_torque = yaw_pid_.update(yaw_error, dt);
	// Set control inputs
	//control_input_.thrust = thrust_command;
	//control_input_.roll_torque = roll_torque;
	//control_input_.pitch_torque = pitch_torque;
	//control_input_.yaw_torque = yaw_torque;
	Inputs control_input_;
	control_input_.thrust = thrust_force;
	control_input_.roll_torque = roll_torque;
	control_input_.pitch_torque = pitch_torque;
	control_input_.yaw_torque = yaw_torque;
	// Step physics 
	physics_.step(state_, control_input_, dt);
	// Logging
	if (log_file_.is_open()) {
		static double elapsed_time = 0.0;
		log_file_ << elapsed_time << ","
			<< state_.x << "," << state_.y << "," << state_.z << ","
			<< state_.vx << "," << state_.vy << "," << state_.vz << ","
			<< state_.roll << "," << state_.pitch << "," << state_.yaw << ","
			<< state_.roll_rate << "," << state_.pitch_rate << "," << state_.yaw_rate << "\n";
		elapsed_time += dt;
	}
}