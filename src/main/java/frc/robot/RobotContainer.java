// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Joystick joystick = new Joystick(0);
  private final Joystick joystickA = new Joystick(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(joystick, joystickA);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(swerveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    new JoystickButton(joystick, 1).onTrue(elevatorSubsystem.levelFourCommand());
    new JoystickButton(joystick, 2).onTrue(elevatorSubsystem.levelOneCommand());
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
