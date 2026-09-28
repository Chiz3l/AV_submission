#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output curr_angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

struct Plant {
    // add whatever state your model needs (velocity, motor-side curr_angle, ...)
    double curr_angle = 0.0;
    double prev_vel = 0.0;
    double delta_theta = 0.0;

    // calculated in the written response 
    const double sys_gain = 1.3;
    const double motor_lag = 0.07179487179;
    const double backlash_magnitude = 2.509;

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output curr_angle, deg
    double step(double u_cmd, double dt) {
        // this equation is explained in the written response
        // first order system equation
        prev_vel += ((sys_gain * u_cmd - prev_vel) / motor_lag) * (dt);
        delta_theta += prev_vel * dt;

        // checking for backlash
        if (delta_theta - curr_angle > backlash_magnitude) {
            curr_angle = delta_theta - backlash_magnitude;
        } else if (delta_theta - curr_angle < -backlash_magnitude) {
            curr_angle = delta_theta + backlash_magnitude;
        }

        // 0.1 deg resolution
        return std::round(curr_angle / 0.1) * 0.1;
    }

    void reset() {
        prev_vel = 0.0;
        delta_theta = 0.0;
        curr_angle = 0.0;
    }
};
