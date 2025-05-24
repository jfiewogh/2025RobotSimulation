// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */

public class CustomPIDController {
    private final PIDController controller;

    // must have same position unit (ex. m and m/s)
    private final double maxError;
    private final double maxSpeed;

    public CustomPIDController(double p, double i, double d, double maxError, double maxSpeed) {
        controller = new PIDController(p, i, d);
        this.maxError = maxError;
        this.maxSpeed = maxSpeed;
    }

    // calculate returns speed
    public double calculateFromSetpoint(double currentPosition, double setpoint) {
        return calculateFromError(setpoint - currentPosition);
    }
    public double calculateFromError(double error) {
        return Math.min(controller.calculate(0, error / maxError), 1) * maxSpeed;
    }
}
