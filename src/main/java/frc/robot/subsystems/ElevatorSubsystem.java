// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Mechanism;
import frc.robot.Constants.MotorSpeed;
import frc.robot.Constants.PIDConstants;
import frc.robot.hardware.CustomPIDController;
import frc.robot.hardware.SimMotor;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ElevatorSubsystem extends SubsystemBase {
  private final SimMotor leftMotor = new SimMotor();
  private final SimMotor rightMotor = new SimMotor();

  private final SwerveSubsystem swerveSubsystem;

  private static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72);
  private static final double kStartingHeightMeters = 0;

  private static final CustomPIDController kElevatorController = new CustomPIDController(
    1, 0, 0, kMaxElevatorHeightMeters, MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond());

  public double speedRotationsPerSecond = 0;

  private double desiredPositionMeters = 0;

  /* Stage Constants */
  private final double stage0Height = 1; // temporary
  private final double stage1Height = 1;
  private final double stage2Height = 1;
  private final double stage3Height = 1;


  /* Simulation Components */
  private Pose3d stage0Pose = Pose3d.kZero;
  private Pose3d stage1Pose = Pose3d.kZero;
  private Pose3d stage2Pose = Pose3d.kZero;
  private Pose3d stage3Pose = Pose3d.kZero;

  private final StructPublisher<Pose3d> stage0Publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Stage0", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> stage1Publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Stage1", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> stage2Publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Stage2", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> stage3Publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Stage3", Pose3d.struct).publish();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    leftMotor.setPositionRotations(Mechanism.kElevator.fromMechanism(kStartingHeightMeters));
    rightMotor.setPositionRotations(leftMotor.getPositionRotations());
  }

  public Command goUp() {
    return new InstantCommand(() -> setSpeedSim(2));
  }

  public Command goDown() {
    return new InstantCommand(
      () -> {
        desiredPositionMeters = 0;
        setSpeedSim(-MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.7);
      }
    );
  }

  public Command levelFourCommand() {
    return new InstantCommand(() -> desiredPositionMeters = Units.feetToMeters(1));
  }

  public Command levelOneCommand() {
    return new InstantCommand(() -> desiredPositionMeters = 0);
  }

  public Command waitForCommand(Command command) {
    return new SequentialCommandGroup(
      command,
      new WaitUntilCommand(this::atSetpoint)
    );
  }

  public void setSpeedSim(double speedRotationsPerSecond) {
    this.speedRotationsPerSecond = speedRotationsPerSecond;
  }

  public double getLeftBarHeightMeters() {
    return Mechanism.kElevator.toMechanism(leftMotor.getPositionRotations());
  }
  public double getRightBarHeightMeters() {
    return Mechanism.kElevator.toMechanism(rightMotor.getPositionRotations());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double motorSpeed = speedRotationsPerSecond;

    if (motorSpeed == 0 && !atSetpoint()) {
      motorSpeed = kElevatorController.calculateFromSetpoint(getLeftBarHeightMeters(), desiredPositionMeters);
    }

    leftMotor.setSpeedRotationsPerSecond(motorSpeed);
    rightMotor.setSpeedRotationsPerSecond(motorSpeed);

    if (getLeftBarHeightMeters() < Units.inchesToMeters(1)) {
      speedRotationsPerSecond = 0;
    }

    stage1Pose = new Pose3d(0, 0, getLeftBarHeightMeters(), Rotation3d.kZero);
    stage2Pose = new Pose3d(0, 0, getLeftBarHeightMeters() + getRightBarHeightMeters(), Rotation3d.kZero);

    stage0Publisher.set(stage0Pose);
    stage1Publisher.set(stage1Pose);
    stage2Publisher.set(stage2Pose);
    stage3Publisher.set(stage3Pose);
  }

  /** Returns whether current position is within 0.5 inches of desired position */
  public boolean atSetpoint() {
    return Math.abs(desiredPositionMeters - getLeftBarHeightMeters()) < Units.inchesToMeters(0.5);
  }
}