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
  private static ElevatorState elevatorState;

  /* Score */
  private boolean score = false;
  private boolean alignedForScore = false;
  private boolean isScoring = false;
  public boolean isScored = false;
  public static ReefScoreState reefScoreState;
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
  public static enum ReefScoreState {
    kL4(IntakeState.kL4, ElevatorState.kL4Score, true),
    kL3(IntakeState.kL3Score, ElevatorState.kL3Score, false),
    kL2(IntakeState.kL2Score, ElevatorState.kL2Score, false),
    kL1(IntakeState.kL1Score, ElevatorState.kL1, true);

    private final IntakeState intakeState;
    private final ElevatorState elevatorState;
    private final boolean isSimultaneous;

    /** isSimultaneous shows whether arm and wrist move at same time */
    private ReefScoreState(IntakeState intakeState, ElevatorState elevatorState, boolean isSimultaneous) {
      this.intakeState = intakeState;
      this.elevatorState = elevatorState;
      this.isSimultaneous = isSimultaneous;
    }

    public IntakeState getIntakeState() {
      return intakeState;
    }
    public ElevatorState getElevatorState() {
      return elevatorState;
    }
    public boolean isSimultaneous() {
      return isSimultaneous;
    }
  }

  /** Creates a new ElevatorIntake. */
  public ElevatorIntake(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, Coral coral) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.coral = coral;
    setState(ElevatorState.kL1, ReefScoreState.kL1);
  }

  public void setState(ElevatorState elevatorState, ReefScoreState reefScoreState) {
    this.elevatorState = elevatorState;
    this.reefScoreState = reefScoreState;
  }

  /** Move elevator up or down */
  public void align() {
    if (!score) {
      if (goUp) {
        goUp = score ? true : false;
        intakeSubsystem.setIntakeState(IntakeState.kStowHorizontal);
      } else {
        goUp = score ? false : true;
        intakeSubsystem.setIntakeState(IntakeState.kStowHorizontal);
      }  
    // If you want to cancel the scoring
    } else if (!isScored) {
      goUp = true;
      alignedForScore = false;
      score = false;
      intakeSubsystem.setIntakeState(IntakeState.kStowHorizontal);
    }
  }

  /** Score coral on reef */
  public void score() {
    if (!alignedForScore) {
      intakeSubsystem.isSimultaneous = reefScoreState.isSimultaneous();
      if (elevatorSubsystem.getDesiredPosition() != 0 && atSetpoint()) {
        goUp = false;
        done = false;
        score = true;
        isScoring = false;
        isScored = false;
        intakeSubsystem.setIntakeState(reefScoreState.getIntakeState());
      } 
    } else if (!isScored) {
      isScoring = true;
      elevatorSubsystem.goDown(reefScoreState.getElevatorState().getPosition());
    } else if (!goDown) {
      goDown = true;
      intakeSubsystem.setIntakeState(IntakeState.kStowVertical);
    }
  }

  /** Intake coral from source (coral still appears even if not at source) */
  public void sourceIntake() {
    if (!isSourceIntake) { 
      isSourceIntake = true;
      intakeSubsystem.setIntakeState(IntakeState.kSource);
      elevatorSubsystem.setElevatorState(ElevatorState.kSource);
      done = false;
    } else {
      isSourceIntake = false;
      intakeSubsystem.setIntakeState(IntakeState.kStowHorizontal);
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
      intakeSubsystem.isSimultaneous = true;
      // If going up
      if (goUp) {
        // If arm and wrist are at right position, go up
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.setElevatorState(elevatorState);
        }
      // If going down
      } else {
        // If arm and wrist are at right position, go down
        if (intakeSubsystem.atSetpoint()) {
          elevatorSubsystem.goDown(0);
        }
      }
    // If scoring coral
    } else {
      // If not aligned
      if (!alignedForScore) {
        // if aligned
        if (intakeSubsystem.atSetpoint()) {
          alignedForScore = true;
        }
      }
      // If not scored
      else if (!isScored) {
        // If scored
        if (isScoring && elevatorSubsystem.atSetpoint()) {
          isScored = true;
          isScoring = false;
          setScoredCoral();
        }
      // If not stowed
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
            alignedForScore = false;
          }
        }
      }
    }
  }

  private void setScoredCoral() {
    scoredCoralPose = coral.getPose();
    scoredCoralPublisher.set(scoredCoralPose);
    coral.visible = false;
  }

  public boolean atSetpoint() {
    return elevatorSubsystem.atSetpoint() && intakeSubsystem.atSetpoint();
  }
}
