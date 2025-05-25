// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class ElevatorIntake extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private boolean goUp = false;
  private ElevatorState elevatorState;

  private boolean score = false;
  public boolean isScored = false;
  private ReefScoreState reefScoreState;

  private boolean done = false;

  /* Scored Coral */
  private final Coral coral;
  private Pose3d scoredCoralPose = Pose3d.kZero;
  private final StructPublisher<Pose3d> scoredCoralPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Scored Coral", Pose3d.struct).publish();

  /* Reef score state */
  public enum ReefScoreState {
    kL4(IntakeState.kL4, ElevatorState.kL4Score),
    kL3(IntakeState.kL3, ElevatorState.kL3Score),
    kL2(IntakeState.kL2, ElevatorState.kL2Score),
    kL1(IntakeState.kL1, ElevatorState.kL1Score);

    private IntakeState intakeState;
    private ElevatorState elevatorState;

    private ReefScoreState(IntakeState intakeState, ElevatorState elevatorState) {
      this.intakeState = intakeState;
      this.elevatorState = elevatorState;
    }

    public IntakeState getIntakeState() {
      return intakeState;
    }

    public ElevatorState getElevatorState() {
      return elevatorState;
    }
  }

  /** Creates a new Score. */
  public ElevatorIntake(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, Coral coral) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.coral = coral;
    setState(ElevatorState.kL4, ReefScoreState.kL4);
  }

  public void setState(ElevatorState elevatorState, ReefScoreState reefScoreState) {
    this.elevatorState = elevatorState;
    this.reefScoreState = reefScoreState;
  }

  public void align() {
    if (goUp) {
      goUp = score ? true : false;
      intakeSubsystem.setIntakeState(IntakeState.kStow2);
    } else {
      goUp = score ? false : true;
      intakeSubsystem.setIntakeState(IntakeState.kStow2);
    }  
    score = false;
    done = false;
  }

  public void score() {
    if (atSetpoint()) {
      done = false;
      score = true;
      isScored = false;
      intakeSubsystem.setIntakeState(reefScoreState.getIntakeState());
    }
  }

  @Override
  public void periodic() {
    if (!score) {
      if (goUp) {
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.setElevatorState(elevatorState);
        }
      } else {
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.goDown(0);
        }
      }
    } else if (!isScored) {
      if (intakeSubsystem.atSetpoint()) {
        elevatorSubsystem.goDown(reefScoreState.getElevatorState().getPosition());
        if (elevatorSubsystem.atSetpoint()) {
          isScored = true;
          createScoredCoral();
        }
      }
    } else if (!done) {
      intakeSubsystem.setIntakeState(IntakeState.kStow1);
      if (intakeSubsystem.atSetpoint()) {
        elevatorSubsystem.goDown(0);
        if (elevatorSubsystem.atSetpoint()) {
          done = true;
        }
      }
    }
  }

  public void createScoredCoral() {
    scoredCoralPose = coral.getPose();
    scoredCoralPublisher.set(scoredCoralPose);
    coral.visible = false;
  }

  public boolean atSetpoint() {
    return elevatorSubsystem.atSetpoint() && intakeSubsystem.atSetpoint();
  }
}
