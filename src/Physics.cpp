#include "Physics.h"
#include "Drone.h"
#include <cmath>

class PhysicsEngine;


PhysicsEngine::PhysicsEngine(double mass)
	: mass_(mass), inertia_{0.0, 0.0, -9.81}, Ixx_(0.02), Iyy_(0.02), Izz_(0.04) {
}

static void bodyToWorld(double thrust, double roll, double pitch, double yaw, std::array<double, 3>& out) {

	//thrust acts along body z axis (down negative in body frame
	// Convert to world coordinates using rotation (roll, pitch, yaw)
	//Roation matrix R = R_yaw * R_pitch * R_roll
	double cr = cos(roll);
	double sr = sin(roll);
	double cp = cos(pitch);
	double sp = sin(pitch);
	double cy = cos(yaw);
	double sy = sin(yaw);

	//Body z axis in world frame (third column of R)
	//Using small matrix math:

	double world_accel[3] = { 0.0, 0.0, 0.0 };

	double body_accel[3] = { 0.0, 0.0, };

	world_accel[0] = (cp * cy) * body_accel[0] + (sr * sp * cy - cr * sy) * body_accel[1] + (cr * sp * cy + sr * sy) * body_accel[2];
	world_accel[1] = (cp * sy) * body_accel[0] + (sr * sp * sy + cr * cy) * body_accel[1] + (cr * sp * sy - sr * cy) * body_accel[2];
	world_accel[2] = (-sp) * body_accel[0] + (sr * cp) * body_accel[1] + (cr * cp) * body_accel[2];

	double bz_x = sy * sp * cr + cy * sr;
	double bz_y = -cy * sp * cr + sy * sr;
	double bz_z = cp * cr;
	
	//thrust is along -body z in many conventions; here we assume thrust positive upward in world: multiply by bz vector
	out[0] = thrust * bz_x;
	out[1] = thrust * bz_y;
	out[2] = thrust * bz_z;
}

void PhysicsEngine::step(State &state, const Inputs& input, double dt) {
	if (dt <= 0.0) {
		return;
	}
	// Compute accelerations
	std::array<double, 3> thrust_world;
	bodyToWorld(input.thrust, state.orientation[0], state.orientation[1], state.orientation[2], thrust_world);
	//bodyToWorld(input.thrust / mass_, state.roll, state.pitch, state.yaw, thrust_world);
	// Linear acceleration
	state.acceleration[0] = (thrust_world[0] / mass_) + inertia_[0];
	state.acceleration[1] = (thrust_world[1] / mass_) + inertia_[1];
	state.acceleration[2] = (thrust_world[2] / mass_) + inertia_[2];

	// Semi-implicit Euler integration for position and velocity
	state.vx += state.acceleration[0] * dt;
	state.vy += state.acceleration[1] * dt;
	state.vz += state.acceleration[2] * dt;

	state.x += state.vx * dt;
	state.y += state.vy * dt;
	state.z += state.vz * dt;
	if (state.z < 0.0) {
		state.z = 0.0;
		state.vz = 0.0;
	} // Prevent going below ground

	//Rotate to body frame for angular acceleration calculation
	//Angular acceleration = torque / inertia
	//Angular acceleration

	state.angular_acceleration[0] = input.roll_torque / Ixx_;
	state.angular_acceleration[1] = input.pitch_torque / Iyy_;
	state.angular_acceleration[2] = input.yaw_torque / Izz_;

	state.angular_velocity[0] += state.angular_acceleration[0] * dt;
	state.angular_velocity[1] += state.angular_acceleration[1] * dt;
	state.angular_velocity[2] += state.angular_acceleration[2] * dt;

	state.orientation[0] += state.angular_velocity[0] * dt;
	state.orientation[1] += state.angular_velocity[1] * dt;
	state.orientation[2] += state.angular_velocity[2] * dt;
}



