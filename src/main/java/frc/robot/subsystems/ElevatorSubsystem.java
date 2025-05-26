// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mechanism;
import frc.robot.Constants.MotorSpeed;
import frc.robot.hardware.CustomPIDController;
import frc.robot.hardware.SimMotor;


/* This subsystem is used when you want to move both the elevator and intake for a command */

public class ElevatorSubsystem extends SubsystemBase {
  /* Height Constants */
  private static final double stage1HeightMeters = 0.662178;
  private static final double stage2HeightMeters = 0.655828;
  private static final double stage3HeightMeters = 0.567614;

  private static final double kMaxElevatorHeightMeters = stage1HeightMeters + stage2HeightMeters + stage3HeightMeters;

  private static final double kStartingHeightMeters = 0;
 
  /* Motor and PID */
  private final SimMotor elevatorMotor = new SimMotor(); // represent both motors using one

  private static final CustomPIDController kElevatorController = new CustomPIDController(
    4, 0, 0.1, kMaxElevatorHeightMeters, MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.8);

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

  /* Speed and Position */

  private double constantSpeedRotationsPerSecond = 0;

  private double desiredPositionMeters = 0;

  private boolean constantSpeed = false;
  private boolean goDown = false;

  /* Elevator States */

  public enum ElevatorState {
    kSource(0.55),

    kL1(0.2),
    kL2(0.6),
    kL3(0.8),
    kL4(kMaxElevatorHeightMeters),

    kL1Score(kL1.getPosition()),
    kL2Score(kL2.getPosition()),
    kL3Score(kL3.getPosition()),
    kL4Score(kL4.getPosition() - 0.37);

    private double position;

    private ElevatorState(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor.setPositionRotations(Mechanism.kElevator.fromMechanism(kStartingHeightMeters));
  }

  public Command goUp() {
    return new InstantCommand(() -> setConstantSpeed(2));
  }

  public void goDown(double desiredPositionMeters) {
    this.desiredPositionMeters = desiredPositionMeters;
    setConstantSpeed(-MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.6);
  }

  public Command goDownCommand(double desiredPositionMeters) {
    return new InstantCommand(
      () -> {
        this.desiredPositionMeters = desiredPositionMeters;
        setConstantSpeed(-MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.7);
      }
    );
  }

  /** Set desired position to level four */
  public void moveLevelFour() {
    setDesiredPosition(2.1);
  }
  public Command moveLevelFourCommand() {
    return new InstantCommand(this::moveLevelFour);
  }

  /** Set desired position to level one */
  public Command moveLevelOneCommand() {
    return new InstantCommand(() -> setDesiredPosition(0));
  }

  /** Set constant speed */
  private void setConstantSpeed(double speedRotationsPerSecond) {
    this.constantSpeedRotationsPerSecond = speedRotationsPerSecond;
    constantSpeed = true;
    goDown = desiredPositionMeters - getElevatorHeightMeters() < 0;
  }
  /** Set desired position in meters */
  public void setDesiredPosition(double position) {
    goDown = position - getElevatorHeightMeters() < 0;
    if (goDown) {
      goDown(position);
    } else {
      desiredPositionMeters = position;
    }
  }

  /** Set elevator state */
  public void setElevatorState(ElevatorState elevatorState) {
    setDesiredPosition(elevatorState.getPosition());
  }


  /** Get the pose with the z position axis only, everything else is 0 */
  public Pose3d getPoseFromZ(double z) {
    return new Pose3d(new Translation3d(0, 0, z), Rotation3d.kZero);
  }


  private double getElevatorHeightMeters() {
    return Mechanism.kElevator.toMechanism(elevatorMotor.getPositionRotations());
  }

  private double previousElevatorHeightMeters = getElevatorHeightMeters();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    /* Stop constant speed when
     * at setpoint OR
     * the velocity is in the opposite direction of the error
     * (prevents elevator from continuing past the desired position)
    */
    if (constantSpeed && (atSetpoint() || Math.signum(constantSpeedRotationsPerSecond) != Math.signum(desiredPositionMeters - getElevatorHeightMeters()))) {
      constantSpeed = false;
      constantSpeedRotationsPerSecond = 0;
      if (desiredPositionMeters == 0) {
        constantSpeedRotationsPerSecond = 0;
        stage1Pose = Pose3d.kZero;
        stage2Pose = Pose3d.kZero;
        stage3Pose = Pose3d.kZero;
      }
    }

    double motorSpeed = constantSpeedRotationsPerSecond;
    if (!constantSpeed && !atSetpoint()) {
      motorSpeed = kElevatorController.calculateFromSetpoint(getElevatorHeightMeters(), desiredPositionMeters);
    }

    elevatorMotor.setSpeedAndUpdatePosition(motorSpeed);

    double elevatorHeightMeters = getElevatorHeightMeters();

    double elevatorHeightChange = elevatorHeightMeters - previousElevatorHeightMeters;
    previousElevatorHeightMeters = elevatorHeightMeters;

    double stage2RelativeHeight = stage2Pose.getZ() - stage1Pose.getZ();
    double stage3RelativeHeight = stage3Pose.getZ() - stage2Pose.getZ();

    // speed is positive, going up
    if (motorSpeed > 0) {
      // only first stage moves
      if (stage1Pose.getZ() != stage1HeightMeters) {
        stage1Pose = getPoseFromZ(Math.min(stage1Pose.getZ() + elevatorHeightChange, stage1HeightMeters));
        stage2Pose = getPoseFromZ(stage1Pose.getZ() + stage2RelativeHeight);
        stage3Pose = getPoseFromZ(stage2Pose.getZ() + stage3RelativeHeight);
      // only second stage moves
      } else if (stage2Pose.getZ() != stage1HeightMeters + stage2HeightMeters) {
        stage2Pose = getPoseFromZ(Math.min(stage2Pose.getZ() + elevatorHeightChange, stage1HeightMeters + stage2HeightMeters));
        stage3Pose = getPoseFromZ(stage2Pose.getZ() + stage3RelativeHeight);
      // only third stage moves
      } else {
        stage3Pose = getPoseFromZ(Math.min(stage3Pose.getZ() + elevatorHeightChange, stage1HeightMeters + stage2HeightMeters + stage3HeightMeters));
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
    return goDown && elevatorHeightMeters < desiredPositionMeters + Units.inchesToMeters(1)
    || !goDown && elevatorHeightMeters > desiredPositionMeters - Units.inchesToMeters(1);
  }

  /* Get Poses */
  public Pose3d getStage3Pose() {
    return stage3Pose;
  }

  public double getDesiredPosition() {
    return desiredPositionMeters;
  }
}