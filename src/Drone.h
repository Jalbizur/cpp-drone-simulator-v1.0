#pragma once
#include "Physics.h"
#include "PID.h"
#include <fstream>
#include <string>

class Drone {
public:
	Drone();
	void reset();
	void setMass(double mass);
	void update(double dt); // step 1 physics with controller
	void setTargetAltitude(double altitude);
	void setTargetAngles(double roll, double pitch, double yaw);
	const State& state() const;
	void enableLogging(const std::string& filename);

private:
	PhysicsEngine physics_;
	State state_;
	//ControlInput control_input_;
	PID altitude_pid_;
	PID roll_pid_;
	PID pitch_pid_;
	PID yaw_pid_;
	double target_altitude_;
	double target_roll_, target_pitch_, target_yaw_;
	std::ofstream log_file_;
};
