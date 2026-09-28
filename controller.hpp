#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.


#include "controller_interface.hpp"

class Controller : public IController {
private:
    double kp = 3.5;
    double ki = 0.8;
    double kd = 0.1;

    double integral = 0.0;
    double prev_measured = 0.0;
    double filtered_derivative = 0.0;
    bool started = false;

public:
    double update(double target, double measured, double dt) override {
        // getting an initial measurement
        if (!started) {
            prev_measured = measured;
            started = true;
        }

        double error = target - measured;

        // derivative on measurement with low pass filter that uses a moving average
        double raw_d = -(measured - prev_measured) / dt; // done to avoid any derivative kick
        prev_measured = measured;
        double alpha = dt / (dt + 0.03); // LPF smoothing
        filtered_derivative += alpha * (raw_d - filtered_derivative);

        // conditional integration
        if (std::fabs(error) > 0.1 && std::fabs(error) < 5.0) {
            integral += error * dt;
            // clamp integral magnitude
            integral = std::clamp(integral, -10.0, 10.0);
        } else if (std::fabs(error) <= 0.1) {
            // sensor resolution limits
            integral *= 0.95;
        }

        double u_cmd = kp * error + ki * integral + kd * filtered_derivative;

        return u_cmd;
    }

    void reset() override {
        integral = 0.0;
        prev_measured = 0.0;
        filtered_derivative = 0.0;
        started = false;
    }
};