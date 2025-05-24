// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CustomPIDController;
import frc.robot.hardware.SimMotor;

public class IntakeSubsystem extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;

  private final SimMotor armMotor = new SimMotor();
  private final SimMotor wristMotor = new SimMotor();

  private final CustomPIDController armController = new CustomPIDController(
    1, 0, 0, 0.25, 1);
  private final CustomPIDController wristController = new CustomPIDController(
    1, 0, 0, 0.25, 1);

  /* Simulation Components */
  private Pose3d initialArmPose = new Pose3d(0.32385, 0, 0.189207, Rotation3d.kZero);
  private Pose3d initialWristPose = new Pose3d(0.376238, 0, 0.354307, Rotation3d.kZero);

  private final double radius = Math.hypot(
    initialArmPose.getX() - initialWristPose.getX(),
    initialArmPose.getZ() - initialWristPose.getZ());

  private final double radians = Math.atan2(
    initialWristPose.getX() - initialArmPose.getX(),
    initialWristPose.getZ() - initialArmPose.getZ());

  private Pose3d armPose = new Pose3d(initialArmPose.getTranslation(), initialArmPose.getRotation());
  private Pose3d wristPose = new Pose3d(initialWristPose.getTranslation(), initialWristPose.getRotation());

  private final StructPublisher<Pose3d> armPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Arm", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> wristPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Wrist", Pose3d.struct).publish();

  /* Desired Positions */
  private double desiredArmPosition = 0;
  private double desiredWristPosition = 0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
  }

  public void setArmPosition(double position) {
    desiredArmPosition = position;
  }

  public void setWristPosition(double position) {
    desiredWristPosition = position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double elevatorHeightMeters = elevatorSubsystem.getElevatorHeightMeters();

    /* Set Speeds */
    double armMotorSpeed = armController.calculateFromSetpoint(armMotor.getPositionRotations(), desiredArmPosition);
    armMotor.setSpeedAndUpdatePosition(armMotorSpeed);

    double wristMotorSpeed = wristController.calculateFromSetpoint(wristMotor.getPositionRotations(), desiredWristPosition);
    wristMotor.setSpeedAndUpdatePosition(wristMotorSpeed);

    /* Get Poses */
    armPose = new Pose3d(
      new Translation3d(initialArmPose.getX(), 0, elevatorHeightMeters + initialArmPose.getZ()),
      new Rotation3d(0, Units.rotationsToRadians(armMotor.getPositionRotations()), 0));

    double armPitchRadians = armPose.getRotation().getY();

    wristPose = new Pose3d(
      new Translation3d(
        armPose.getX() + radius * Math.sin(radians + armPitchRadians), 
        0,
        armPose.getZ() + radius * Math.cos(radians + armPitchRadians)),
      new Rotation3d(
        0,
        armPitchRadians,
        0)
    );

    /* Set Poses */
    armPublisher.set(armPose);
    wristPublisher.set(wristPose);
  }
}
