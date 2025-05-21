// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ElevatorSubsystem extends SubsystemBase {
  StructPublisher<Pose3d> leftBar = NetworkTableInstance.getDefault()
    .getStructTopic("LeftBar", Pose3d.struct).publish();
  
  StructPublisher<Pose3d> rightBar = NetworkTableInstance.getDefault()
    .getStructTopic("RightBar", Pose3d.struct).publish();

  private Pose3d leftBarPose = new Pose3d();
  private Pose3d rightBarPose = new Pose3d();

  private final SparkMax leftMotor = new SparkMax(6, MotorType.kBrushless); 
  private final SparkMax rightMotor = new SparkMax(5, MotorType.kBrushless);

  private final RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  private final SwerveSubsystem swerveSubsystem;

  private static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72);
  private static final double kStartingHeight = 0; // unused should affect motor encoder position

  public double speedRotationsPerSecond = 0;

  private double desiredPositionMeters = kStartingHeight;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    leftMotorEncoder.setPosition(Mechanism.kElevator.fromMechanism(kStartingHeight));
  }

  public Command goUpCommand() {
    return new InstantCommand(() -> setSpeedSim(2));
  }

  public Command goDownCommand() {
    return new InstantCommand(
      () -> {
        desiredPositionMeters = 0;
        setSpeedSim(-MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond() * 0.7);
      });
  }

  public Command levelFourCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        desiredPositionMeters = Units.feetToMeters(7);
        System.out.println("Lets go");
      }),
      new WaitUntilCommand(this::atSetpoint)
    );
  }

  public Command levelOneCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> desiredPositionMeters = 0),
      new WaitUntilCommand(this::atSetpoint)
    );
  }



  public void setSpeedSim(double speedRotationsPerSecond) {
    this.speedRotationsPerSecond = speedRotationsPerSecond;
  }



  // meters
  public double getLeftBarHeight() {
    return Mechanism.kElevator.toMechanism(leftMotorEncoder.getPosition());
  }
  public double getRightBarHeight() {
    return Mechanism.kElevator.toMechanism(rightMotorEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double speed = speedRotationsPerSecond;

    // System.out.println(desiredPositionMeters);

    // for some elevator moves after done

    if (speed == 0 && !atSetpoint()) {
      double mechanismSpeed = PIDConstants.kElevatorController.calculate(getLeftBarHeight() / kMaxElevatorHeightMeters, desiredPositionMeters / kMaxElevatorHeightMeters);
      double motorSpeed = Mechanism.kElevator.fromMechanism(mechanismSpeed);
      speed = Math.signum(motorSpeed) * Math.min(Math.abs(motorSpeed), MotorSpeed.kVortex.getFreeSpeedRotationsPerSecond());
    }

    leftMotorEncoder.setPosition(leftMotorEncoder.getPosition() + speed * 0.02);
    rightMotorEncoder.setPosition(leftMotorEncoder.getPosition() + speed * 0.02);

    if (getLeftBarHeight() < Units.inchesToMeters(1)) {
      speedRotationsPerSecond = 0;
    }

    Pose2d robotPose = swerveSubsystem.getPose();

    leftBarPose = new Pose3d(robotPose.getX(), robotPose.getY(), getLeftBarHeight(), Rotation3d.kZero);
    rightBarPose = new Pose3d(robotPose.getX(), robotPose.getY(), getRightBarHeight(), Rotation3d.kZero);

    leftBar.set(leftBarPose);
    rightBar.set(rightBarPose);
  }

  public boolean atSetpoint() {

    boolean a = Math.abs(desiredPositionMeters - getLeftBarHeight()) < Units.inchesToMeters(0.5);

    // System.out.println(a + " " + desiredPositionMeters + " " + getLeftBarHeight());

    return a;
  }
}
