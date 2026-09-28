#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>
#include <iostream>

struct Plant {
    // add whatever state your model needs (velocity, motor-side angle, ...)
    double angle = 0.0;
    double velocity = 0.0;    
    // where actuator would be without backlash (motor-side angle)
    double internal_angle = 0.0;

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    double step(double u_cmd, double dt) {
        double deadzone = 0.0;
        double gain = 54.8 / 2.8 / 15;
        double time_constant = 0.08;
        double backlash = 5.0;
        double y_0 = 0;

        double u_eff;
        if (abs(u_cmd) <= deadzone) {
            // deadzone
            u_eff = 0.0;
        } else {
            u_eff = std::copysign(std::abs(u_cmd) - deadzone, u_cmd);
        }
        
        // velocity dynamics
        double velocity_target = gain * u_eff;
        // gradually approach the target according to time constant
        velocity = velocity_target +
            (velocity - velocity_target) * std::exp(-dt / time_constant);

        // integrate: angular velocity = d\theta / dt
        internal_angle += velocity * dt;

        // Actuator-side position continues moving but measured angle only 
        // follows once the internal position reaches one edge of the backlash gap
        // boundaries are +2.5 and -2.5
        double half_backlash = backlash / 2.0;
        if (internal_angle - angle > half_backlash) {
            angle = internal_angle - half_backlash;
        } else if (internal_angle - angle < -half_backlash) {
            angle = internal_angle + half_backlash;
        }

        return std::round(angle / 0.1) * 0.1;  // the sensor reads to 0.1 deg
    }

    void reset() { angle = 0.0; }
};
