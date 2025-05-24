// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.LeftAlignCommand;
import frc.robot.commands.RightAlignCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Joystick realController = new Joystick(2);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Constants.kControllerType, keyboardLeftStick, keyboardRightStick, realController);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(elevatorSubsystem);

  private final Vision vision = new Vision(swerveSubsystem);

  private final LeftAlignCommand leftAlignCommand = new LeftAlignCommand(vision);
  private final RightAlignCommand rightAlignCommand = new RightAlignCommand(vision);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("LeftAlign", leftAlignCommand);
    NamedCommands.registerCommand("RightAlign", rightAlignCommand);
    
    NamedCommands.registerCommand("L4", elevatorSubsystem.waitForCommand(elevatorSubsystem.levelFourCommand()));

    NamedCommands.registerCommand("L1", elevatorSubsystem.waitForCommand(elevatorSubsystem.levelOneCommand()));

    NamedCommands.registerCommand("UpAndDown", new SequentialCommandGroup(
      NamedCommands.getCommand("L4"),
      NamedCommands.getCommand("L1")
    ));

    NamedCommands.registerCommand("ElevatorDown", elevatorSubsystem.goDown());


    // Configure the trigger bindings
    configureBindings();

    // swerveSubsystem.setDefaultCommand(new InstantCommand(
    //   () -> swerveSubsystem.fieldCentricSwerve(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> joystickA.getRawAxis(0)),
    //   swerveSubsystem
    // ));
    // swerveSubsystem2.setDefaultCommand(new InstantCommand(
    //   () -> swerveSubsystem2.fieldCentricSwerve(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> joystickA.getRawAxis(0)),
    //   swerveSubsystem2
    // ));
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
    new JoystickButton(keyboardLeftStick, 1).onTrue(elevatorSubsystem.levelFourCommand());
    new JoystickButton(keyboardLeftStick, 2).onTrue(elevatorSubsystem.goDown());

    new JoystickButton(keyboardLeftStick, 3).onTrue(leftAlignCommand);
    new JoystickButton(keyboardLeftStick, 4).onTrue(rightAlignCommand);
  
    switch (Constants.kControllerType) {
      case KEYBOARD:
        new JoystickButton(keyboardRightStick, 1).onTrue(swerveSubsystem.resetGyroCommand());
        break;
      case XBOX:
        new JoystickButton(realController, 1).onTrue(swerveSubsystem.resetGyroCommand());
        break;
    }
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

    return new PathPlannerAuto("New Auto");
  }
}
