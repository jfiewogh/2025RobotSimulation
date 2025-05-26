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

/* This subsystem is used when you want to move both the elevator and intake for a command */

public class ElevatorIntake extends SubsystemBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  /* Move Elevator and Intake to Position */
  private boolean goUp = false;
  private ElevatorState elevatorState;

  /* Score */
  private boolean score = false;
  public boolean isScored = false;
  private ReefScoreState reefScoreState;
  private boolean goDown = false;
  private boolean done = false;

  /* Intake */
  private boolean isSourceIntake = false;

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
    if (!score) {
      if (goUp) {
        goUp = score ? true : false;
        intakeSubsystem.setIntakeState(IntakeState.kStow2);
      } else {
        goUp = score ? false : true;
        intakeSubsystem.setIntakeState(IntakeState.kStow2);
      }  
    }
  }

  public void score() {
    if (!isScored) {
      if (elevatorSubsystem.getDesiredPosition() != 0 && atSetpoint()) {
        goUp = false;
        done = false;
        score = true;
        isScored = false;
        intakeSubsystem.setIntakeState(reefScoreState.getIntakeState());
      }
    } else if (!goDown) {
      goDown = true;
      intakeSubsystem.setIntakeState(IntakeState.kStow1);
    }
  }

  public void sourceIntake() {
    if (!isSourceIntake) { 
      isSourceIntake = true;
      intakeSubsystem.setIntakeState(IntakeState.kSource);
      elevatorSubsystem.setElevatorState(ElevatorState.kSource);
      done = false;
    } else {
      isSourceIntake = false;
      intakeSubsystem.setIntakeState(IntakeState.kStow2);
      elevatorSubsystem.setDesiredPosition(0);
    }
  }

  @Override
  public void periodic() {
    // If source intake
    if (isSourceIntake) {
      // If intake is aligned and piece is intaked
      if (!done && atSetpoint()) {
        coral.visible = true;
        done = true;
        isSourceIntake = false;
      }
    // If moving elevator up or down
    } else if (!score) {
      // If going up
      if (goUp) {
        // If arm and wrist are at right position, go up
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.setElevatorState(elevatorState);
        }
      // If going down */
      } else {
        // If arm and wrist are at right position, go down
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.goDown(0);
        }
      }
    // If scoring coral
    } else {
      // If scored
      if (!isScored) {
        // If arm and wrist are aligned, go down
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.goDown(reefScoreState.getElevatorState().getPosition());
          // If scored
          if (elevatorSubsystem.atSetpoint()) {
            isScored = true;
            createScoredCoral();
          }
        }
      // If not scored
      } else if (!done) {
        // If arm and wrist are aligned and want to go down, then go down
        if (goDown && intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.goDown(0);
          // If elevator at bottom
          if (elevatorSubsystem.atSetpoint()) {
            done = true;
            score = false;
            isScored = false;
            goDown = false;
          }
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
