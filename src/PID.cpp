#include "PID.h"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double imax)
	: kp_(kp), ki_(ki), kd_(kd), imax_(imax), integrator_(0.0), prev_error_(0.0) {
}

double PID::update(double error, double dt) {
	if(dt <= 0.0) {
		return 0.0;
	}

	integrator_ += error * dt;
	integrator_ = std::clamp(integrator_, -imax_, imax_);
	double derivative = (error - prev_error_) / dt;
	prev_error_ = error;
	return kp_ * error + ki_ * integrator_ + kd_ * derivative;
}

void PID::reset() {
	integrator_ = 0.0;
	prev_error_ = 0.0;
}

void PID::setGains(double kp, double ki, double kd) {
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

void PID::setIntegralMax(double imax) {
	imax_ = imax;
}
