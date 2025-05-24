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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Mechanism;
import frc.robot.Constants.MotorSpeed;
import frc.robot.hardware.CustomPIDController;
import frc.robot.hardware.SimMotor;

/* Current Issues
 * - Elevator height not matching after pressing up and down quickly
 */


public class ElevatorSubsystem extends SubsystemBase {
  private final SimMotor elevatorMotor = new SimMotor(); // represent both motors using one

  private static final double kMaxElevatorHeightMeters = 2.1; // Units.inchesToMeters(72);
  
  private static final double kStartingHeightMeters = 0;

  private static final CustomPIDController kElevatorController = new CustomPIDController(
    15, 0.01, 0, kMaxElevatorHeightMeters, MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.8);

  private double speedRotationsPerSecond = 0;

  private double desiredPositionMeters = 0;

  private boolean constantSpeed = false;
  private boolean goDown = false;

  /* Stage Constants */
  private final double stage0HeightMeters = 0.7; // temporary
  private final double stage1HeightMeters = 0.7;
  private final double stage2HeightMeters = 0.7;
  private final double stage3HeightMeters = 0.7; // unused


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
  public ElevatorSubsystem() {
    elevatorMotor.setPositionRotations(Mechanism.kElevator.fromMechanism(kStartingHeightMeters));
  }

  public Command goUp() {
    return new InstantCommand(() -> setSpeedSim(2));
  }

  public Command goDown() {
    return new InstantCommand(
      () -> {
        desiredPositionMeters = 0;
        setSpeedSim(-MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.6);
      }
    );
  }

  public Command levelFourCommand() {
    return new InstantCommand(() -> setDesiredPosition(2.1));
  }

  public Command levelOneCommand() {
    return new InstantCommand(() -> setDesiredPosition(0));
  }

  public Command waitForCommand(Command command) {
    return new SequentialCommandGroup(
      command,
      new WaitUntilCommand(this::atSetpoint)
    );
  }

  /** Set constant speed */
  public void setSpeedSim(double speedRotationsPerSecond) {
    this.speedRotationsPerSecond = speedRotationsPerSecond;
    constantSpeed = true;
    goDown = desiredPositionMeters - getElevatorHeightMeters() < 0;
  }
  /** Set desired position in meters */
  public void setDesiredPosition(double position) {
    desiredPositionMeters = position;
    goDown = desiredPositionMeters - getElevatorHeightMeters() < 0;
  }


  /** Get the pose with the z position axis only, everything else is 0 */
  public Pose3d getPoseFromZ(double z) {
    return new Pose3d(new Translation3d(0, 0, z), Rotation3d.kZero);
  }


  public double getElevatorHeightMeters() {
    return Mechanism.kElevator.toMechanism(elevatorMotor.getPositionRotations());
  }

  private double previousElevatorHeightMeters = getElevatorHeightMeters();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elevatorHeightMeters = getElevatorHeightMeters();

    /* Stop constant speed when
     * at setpoint OR
     * the velocity is in the opposite direction of the error
     * (prevents elevator from continuing past the desired position)
    */
    if (constantSpeed && (atSetpoint() || Math.signum(speedRotationsPerSecond) != Math.signum(desiredPositionMeters - elevatorHeightMeters))) {
      constantSpeed = false;
      speedRotationsPerSecond = 0;
      if (desiredPositionMeters == 0) {
        speedRotationsPerSecond = 0;
        stage1Pose = Pose3d.kZero;
        stage2Pose = Pose3d.kZero;
        stage3Pose = Pose3d.kZero;
      }
    }

    double motorSpeed = speedRotationsPerSecond;
    if (!constantSpeed && !atSetpoint()) {
      motorSpeed = kElevatorController.calculateFromSetpoint(elevatorHeightMeters, desiredPositionMeters);
    }

    elevatorMotor.setSpeedAndUpdatePosition(motorSpeed);

    double elevatorHeightChange = elevatorHeightMeters - previousElevatorHeightMeters;
    previousElevatorHeightMeters = elevatorHeightMeters;

    double stage2RelativeHeight = stage2Pose.getZ() - stage1Pose.getZ();
    double stage3RelativeHeight = stage3Pose.getZ() - stage2Pose.getZ();

    // speed is positive, going up
    if (motorSpeed > 0) {
      // only first stage moves
      if (stage1Pose.getZ() != stage0HeightMeters) {
        stage1Pose = getPoseFromZ(Math.min(stage1Pose.getZ() + elevatorHeightChange, stage0HeightMeters));
        stage2Pose = getPoseFromZ(stage1Pose.getZ() + stage2RelativeHeight);
        stage3Pose = getPoseFromZ(stage2Pose.getZ() + stage3RelativeHeight);
      // only second stage moves
      } else if (stage2Pose.getZ() != stage0HeightMeters + stage1HeightMeters) {
        stage2Pose = getPoseFromZ(Math.min(stage2Pose.getZ() + elevatorHeightChange, stage0HeightMeters + stage1HeightMeters));
        stage3Pose = getPoseFromZ(stage2Pose.getZ() + stage3RelativeHeight);
      // only third stage moves
      } else {
        stage3Pose = getPoseFromZ(Math.min(stage3Pose.getZ() + elevatorHeightChange, stage0HeightMeters + stage1HeightMeters + stage2HeightMeters));
      }
    }
    // speed is negative, going down
    else if (motorSpeed < 0) {
      // only first stage moves
      if (stage1Pose.getZ() != 0) {
        stage1Pose = getPoseFromZ(Math.max(stage1Pose.getZ() + elevatorHeightChange, 0));
        stage2Pose = getPoseFromZ(stage1Pose.getZ() + stage2RelativeHeight);
        stage3Pose = getPoseFromZ(stage2Pose.getZ() + stage3RelativeHeight);
      // only second stage moves
      } else if (stage2Pose.getZ() != 0) {
        stage2Pose = getPoseFromZ(Math.max(stage2Pose.getZ() + elevatorHeightChange, 0));
        stage3Pose = getPoseFromZ(stage2Pose.getZ() + stage3RelativeHeight);
      // only third stage moves
      } else {
        stage3Pose = getPoseFromZ(Math.max(stage3Pose.getZ() + elevatorHeightChange, 0));
      }
    }

    stage0Publisher.set(stage0Pose);
    stage1Publisher.set(stage1Pose);
    stage2Publisher.set(stage2Pose);
    stage3Publisher.set(stage3Pose);
  }

  /** Returns whether current position is within 0.5 inches of desired position */
  public boolean atSetpoint() {
    double elevatorHeightMeters = getElevatorHeightMeters();
    return goDown && elevatorHeightMeters < desiredPositionMeters + Units.inchesToMeters(0.5)
    || !goDown && elevatorHeightMeters > desiredPositionMeters - Units.inchesToMeters(0.5);
  }
}