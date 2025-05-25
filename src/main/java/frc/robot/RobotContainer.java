// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.vision.LeftAlignCommand;
import frc.robot.commands.vision.RightAlignCommand;
import frc.robot.commands.vision.SourceAlignCommand;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ElevatorIntake.ReefScoreState;
import frc.robot.subsystems.ElevatorIntake;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick keyboardLeftStick = new Joystick(0);
  private final Joystick keyboardRightStick = new Joystick(1);
  private final Joystick keyboardThirdStick = new Joystick(2);

  private final Joystick realController = new Joystick(2);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Constants.kControllerType, keyboardLeftStick, keyboardRightStick, realController);
  private final Vision vision = new Vision(swerveSubsystem);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(elevatorSubsystem);
  private final Coral coral = new Coral(swerveSubsystem, intakeSubsystem);
  private final ElevatorIntake elevatorIntake = new ElevatorIntake(elevatorSubsystem, intakeSubsystem, coral);

  private final LeftAlignCommand leftAlignCommand = new LeftAlignCommand(vision);
  private final RightAlignCommand rightAlignCommand = new RightAlignCommand(vision);
  private final SourceAlignCommand sourceAlignCommand = new SourceAlignCommand(vision);

  private final Command groundIntakeCommand = new SequentialCommandGroup(
    intakeSubsystem.setIntakeStateCommand(IntakeState.kGround),
    new WaitUntilCommand(intakeSubsystem::atSetpoint),
    new InstantCommand(() -> coral.visible = true)
  );
  private final Command torchIntakeCommand = new SequentialCommandGroup(
    intakeSubsystem.setIntakeStateCommand(IntakeState.kTorch),
    new WaitUntilCommand(intakeSubsystem::atSetpoint),
    new InstantCommand(() -> coral.visible = true)
  );
  private final Command sourceIntakeCommand = new SequentialCommandGroup(
    intakeSubsystem.setIntakeStateCommand(IntakeState.kSource),
    new InstantCommand(() -> elevatorSubsystem.setElevatorState(ElevatorState.kSource)),
    new WaitUntilCommand(elevatorIntake::atSetpoint),
    new InstantCommand(() -> coral.visible = true)
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("LeftAlign", leftAlignCommand);
    NamedCommands.registerCommand("RightAlign", rightAlignCommand);
    
    NamedCommands.registerCommand("L4", new SequentialCommandGroup(
      new InstantCommand(() -> elevatorIntake.setState(ElevatorState.kL4, ReefScoreState.kL4)),
      fullAutoScoreCommand()
    ));

    NamedCommands.registerCommand("L1", elevatorSubsystem.moveLevelOneCommand());

    NamedCommands.registerCommand("UpAndDown", new SequentialCommandGroup(
      NamedCommands.getCommand("L4"),
      NamedCommands.getCommand("L1")
    ));

    NamedCommands.registerCommand("ElevatorDown", new SequentialCommandGroup(
      elevatorSubsystem.goDownCommand(0),
      new WaitUntilCommand(0)));

    NamedCommands.registerCommand("TorchIntake", torchIntakeCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* Elevator and Intake */

    // Up or Down
    new JoystickButton(keyboardLeftStick, 1).onTrue(
      new InstantCommand(() -> elevatorIntake.align()));
    // Align Score, Then Score
    new JoystickButton(keyboardLeftStick, 2).onTrue(
      new InstantCommand(() -> elevatorIntake.score()));

    /* Intake and Align */

    // Ground Intake
    new JoystickButton(keyboardLeftStick, 3)
      .onTrue(groundIntakeCommand)
      .onFalse(new InstantCommand(() -> intakeSubsystem.setIntakeState(IntakeState.kStow1)));
    // Torch 
    /*
    new JoystickButton(keyboardRightStick, 2)
      .onTrue(torchIntakeCommand)
      .onFalse(new InstantCommand(() -> intakeSubsystem.setIntakeState(IntakeState.kStow1)));
      */
    // Source Intake
    new JoystickButton(keyboardLeftStick, 4)
      .onTrue(sourceIntakeCommand);

    // Align
    new JoystickButton(keyboardRightStick, 1).onTrue(leftAlignCommand);
    new JoystickButton(keyboardRightStick, 2).onTrue(rightAlignCommand);
    new JoystickButton(keyboardRightStick, 3).onTrue(sourceAlignCommand);
    // future: barge align

    /* Elevator Choices */
    new JoystickButton(keyboardThirdStick, 4).onTrue(new InstantCommand(() -> 
      elevatorIntake.setState(ElevatorState.kL4, ReefScoreState.kL4)));
    new JoystickButton(keyboardThirdStick, 1).onTrue(new InstantCommand(() -> 
      elevatorIntake.setState(ElevatorState.kL3, ReefScoreState.kL3)));
    new JoystickButton(keyboardThirdStick, 5).onTrue(new InstantCommand(() -> 
      elevatorIntake.setState(ElevatorState.kL2, ReefScoreState.kL2)));
    new JoystickButton(keyboardThirdStick, 2).onTrue(new InstantCommand(() -> 
      elevatorIntake.setState(ElevatorState.kL1, ReefScoreState.kL1)));

    switch (Constants.kControllerType) {
      case KEYBOARD:
        // new JoystickButton(keyboardRightStick, 1).onTrue(swerveSubsystem.resetGyroCommand());
        break;
      case XBOX:
        new JoystickButton(realController, 1).onTrue(swerveSubsystem.resetGyroCommand());
        break;
    }
  }

  public Command fullAutoScoreCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> elevatorIntake.align()),
      new WaitUntilCommand(elevatorIntake::atSetpoint),
      new InstantCommand(() -> elevatorIntake.score()),
      new WaitUntilCommand(() -> elevatorIntake.isScored),
      new WaitUntilCommand(elevatorIntake::atSetpoint)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //   SwerveSubsystem.maxDriveSpeedMetersPerSecond,
    //   SwerveSubsystem.maxDriveAccelerationMetersPerSecondSquared
    // );

    // ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    // waypoints.add(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    // waypoints.add(new Pose2d(new Translation2d(5, 10), Rotation2d.fromDegrees(50)));
    // waypoints.add(new Pose2d(new Translation2d(20, -5), Rotation2d.fromDegrees(90)));
    // waypoints.add(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));

    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   waypoints, 
    //   trajectoryConfig
    // );

    // // An example command will be run in autonomous
    // return new AutoCommand(swerveSubsystem, trajectory);

    return new PathPlannerAuto("3 Coral Auto");
  }
}
