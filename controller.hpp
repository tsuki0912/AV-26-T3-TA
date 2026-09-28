#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.


#include "controller_interface.hpp"
#include <iostream>

class Controller : public IController {
private:
    double integral = 0.0;
    double prev_error = 0.0;
    double prev_measured_angle = 0.0;

public:
    double update(double target, double measured, double dt) override {
        (void)dt;
        double error = target - measured;
        double kp = 6.5; 
        double ki = 0.1;
        double kd = 0.32;
        double integral_limit = 3.0;

        double P = kp * error;

        integral += error * dt;
        // limiting accumulated error 
        integral = std::clamp(integral, -integral_limit, integral_limit);
        double I = ki * integral;

        // double derivative = (error - prev_error) / dt;
        // double D = kd * derivative;
        double derivative = (measured - prev_measured_angle) / dt;
        double D = -kd * derivative;

        prev_measured_angle = measured;
    
        double output = P + I + D;
        //prev_error = error;
        return output;
    }

    void reset() override {
        integral = 0.0;
        prev_error = 0.0;
        prev_measured_angle = 0.0;
    }
};
