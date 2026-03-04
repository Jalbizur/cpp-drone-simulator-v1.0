#pragma once
#include <array>


struct State {
	// Positions (meters)

	double x = 0.0, y = 0.0, z = 0.0;

	double vx = 0.0, vy = 0.0, vz = 0.0; // Velocities (meters/second)

	double roll, pitch, yaw; // Orientations (radians)

	double roll_rate, pitch_rate, yaw_rate; // Angular velocities (radians/second)

	double p, q, r; // Body frame angular velocities (radians/second)

	std::array<double, 3> orientation{0.0, 0.0, 0.0};

	std::array<double, 3> acceleration{0.0, 0.0, 0.0};

	std::array<double, 3> angular_velocity{ 0.0, 0.0, 0.0 };

	std::array<double, 3> angular_acceleration{ 0.0, 0.0, 0.0 };


};

struct Inputs {
	// collective thrust (Newtons) and body torques (Newton-meters)

	double thrust = 0.0;

	double roll_torque = 0.0, pitch_torque = 0.0, yaw_torque = 0.0;
};

class PhysicsEngine {
public:
	PhysicsEngine(double mass = 1.0);
	void step(State& s, const Inputs& u, double dt);
	double mass() const { return mass_; }
private:

	double mass_;
	std::array<double, 3> inertia_; // Moment of inertia around x, y,
	// and z axes (kg*m^2)
	double Ixx_, Iyy_, Izz_;
};