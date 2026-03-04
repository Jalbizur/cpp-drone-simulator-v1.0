#include <iostream>
#include <chrono>
#include <thread>
#include "Drone.h"

int main() {

	bool running = true;
	//SDL_Event event;
	Drone drone;
	drone.setTargetAltitude(2.0);
	drone.enableLogging("telemetry.csv");

	const double dt = 0.01; // 100 Hz sim
	double t = 0.0;
	const double sim_time = 10.0; //seconds

	std::cout << "Starting simulation for " << sim_time << "s (dt=" << dt << ")\n";
	while (t < sim_time)
	{
		// Simple manual disturbance between 3s and 4s tilt target
		if (t > 3.0 && t < 4.0)
		{
			drone.setTargetAngles(0.2, 0.1, 0.0); // small roll/pitch setpoint
		}
		else
		{
			drone.setTargetAngles(0.0, 0.0, 0.0);
		}

		drone.update(dt);
		if (static_cast<int>(t * 10) % 10 == 0)
		{
			const auto& s = drone.state();
			std::cout << "t=" << t << "s, z=" << s.z << "m, vz=" << s.vz << "m/s, roll=" << s.roll << "rad, pitch=" << s.pitch << "rad\n";
		}

		t += dt;
		std::this_thread::sleep_for(std::chrono::milliseconds(2)); // simulate real-time and be nice to CPU
	}



	std::cout << "Simulation complete, telemetry.csv written.\n";
	return 0;
}