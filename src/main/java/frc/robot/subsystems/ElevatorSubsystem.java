// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

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

  
  
  private static final double elevatorGearRatio = 1.0 / 10.0; // don't what it is exactly


  private final SwerveSubsystem swerveSubsystem;

  private final PIDController positionController = new PIDController(1.5, 0, 1);


  double speedRotationsPerSecond = 0;


  Mechanism2d mechanism = new Mechanism2d(2, 4);

  double desiredPositionRotations = 0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(SwerveSubsystem swerveSubsystem) {
    mechanism.getRoot("this", 0.2, 0).append(new MechanismLigament2d("extend", 1, 90));
    mechanism.getRoot("other", -0.2, 0).append(new MechanismLigament2d("expand", 1, 90));
 
    this.swerveSubsystem = swerveSubsystem;
  }

  public Command goUp() {
    return new InstantCommand(() -> setSpeedSim(2));
  }

  public Command levelFourCommand() {
    return new InstantCommand(() -> desiredPositionRotations = 100);
  }

  public Command levelOneCommand() {
    return new InstantCommand(() -> desiredPositionRotations = 10);
  }


  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void setSpeedSim(double speedRotationsPerSecond) {
    this.speedRotationsPerSecond = speedRotationsPerSecond;
  }

  public void setPositionSim(double desiredPositionRotations) {
    this.speedRotationsPerSecond = positionController.calculate(leftMotorEncoder.getPosition(), desiredPositionRotations);
  }



  // meters
  public double getLeftBarHeight() {
    return leftMotorEncoder.getPosition() * elevatorGearRatio;
  }
  public double getRightBarHeight() {
    return rightMotorEncoder.getPosition() * elevatorGearRatio;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    speedRotationsPerSecond = positionController.calculate(leftMotorEncoder.getPosition(), desiredPositionRotations);
    speedRotationsPerSecond = Math.signum(speedRotationsPerSecond) * Math.min(Math.abs(speedRotationsPerSecond), 20);

    leftMotorEncoder.setPosition(leftMotorEncoder.getPosition() + speedRotationsPerSecond * 0.02);
    rightMotorEncoder.setPosition(leftMotorEncoder.getPosition() + speedRotationsPerSecond * 0.02);

    Pose2d robotPose = swerveSubsystem.getPose();

    leftBarPose = new Pose3d(robotPose.getX(), robotPose.getY(), getLeftBarHeight(), Rotation3d.kZero);
    rightBarPose = new Pose3d(robotPose.getX(), robotPose.getY(), getRightBarHeight(), Rotation3d.kZero);

    leftBar.set(leftBarPose);
    rightBar.set(rightBarPose);
  }
}
