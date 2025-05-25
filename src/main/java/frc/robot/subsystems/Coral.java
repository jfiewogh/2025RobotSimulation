// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Coral extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private Pose3d pose = Pose3d.kZero;

  public boolean visible = true;

  private final StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Coral", Pose3d.struct).publish();

  /** Creates a new Coral. */
  public Coral(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (visible) {
      Pose3d wristPose = intakeSubsystem.getWristPose();
      pose = new Pose3d(swerveSubsystem.getPose()).plus(new Transform3d(wristPose.getTranslation(), wristPose.getRotation()));
      pose = pose.plus(new Transform3d(new Translation3d(0.13, -0.01, 0), new Rotation3d(0, Math.PI / 2, 0)));
    } else {
      pose = Pose3d.kZero;
    }

    publisher.set(pose);
  }

  public Pose3d getPose() {
    return pose;
  } 
}
