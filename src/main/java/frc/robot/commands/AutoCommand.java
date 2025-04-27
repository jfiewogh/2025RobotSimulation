// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand extends Command {
  /** Creates a new AutoCommand. */
  private final SwerveSubsystem swerveSubsystem;
    private final Trajectory trajectory;

    private final Timer timer = new Timer();

    private static final PIDController X_CONTROLLER = new PIDController(1, 0, 0.2);
    private static final PIDController Y_CONTROLLER = new PIDController(1, 0, 0.2);
    private static final PIDController THETA_CONTROLLER = new PIDController(1, 0, 0.2);

    public AutoCommand(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        this.swerveSubsystem = swerveSubsystem;
        this.trajectory = trajectory;
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSubsystem.updateOdometry();

        double currentTime = timer.get();

        State desiredState = trajectory.sample(currentTime); 
        Pose2d desiredPose = desiredState.poseMeters;

        Pose2d currentPose = swerveSubsystem.getPose();

        double xSpeed = getXSpeed(currentPose.getX(), desiredPose.getX());
        double ySpeed = getYSpeed(currentPose.getY(), desiredPose.getY());
        double rotationSpeed = getRotationSpeed(currentPose.getRotation(), desiredPose.getRotation());

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

        swerveSubsystem.robotCentricSwerve(xSpeed, ySpeed, rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public double getXSpeed(double currentX, double desiredX) {
        return X_CONTROLLER.calculate(currentX, desiredX);
    }

    public double getYSpeed(double currentY, double desiredY) {
        return Y_CONTROLLER.calculate(currentY, desiredY);
    }

    public double getRotationSpeed(Rotation2d currentRotation, Rotation2d desiredRotation) {
        double errorRadians = desiredRotation.getRadians() - currentRotation.getRadians();
        return THETA_CONTROLLER.calculate(0, errorRadians / Math.PI * 2);
    }
}
