#pragma once

class PID {
	
public:
	PID(double kp = 0, double ki = 0, double kd = 0, double imax = 1e6);
	double update(double error, double dt);

	void reset();
	void setGains(double kp, double ki, double kd);
	void setIntegralMax(double imax);

private:

	double kp_, ki_, kd_;
	double integrator_;
	double imax_;
	double prev_error_;

	double compute(double setpoint, double measured_value, double dt) {
		double error = setpoint - measured_value;
		integrator_ += error * dt;
		double derivative = (error - prev_error_) / dt;
		prev_error_ = error;
		return kp_ * error + ki_ * integrator_ + kd_ * derivative;
	}
};
