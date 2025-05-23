// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTag;
import frc.robot.Constants.VisionConstants;
import frc.robot.hardware.CustomPIDController;

public class Vision extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;

  private Pose2d desiredPosition;

  private Pose2d currentPosition;

  private final PIDController translationController = new PIDController(2, 0, 0);

  private final CustomPIDController rotationController = new CustomPIDController(
    2, 0, 0, Math.PI / 2, SwerveSubsystem.kPhysicalMaxRotationSpeedRadiansPerSecond);

  /** Creates a new Vision. */
  public Vision(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentPosition = swerveSubsystem.getPose();
  }

  public void leftReefAlign() {
    AprilTag closestAprilTag = getClosestReefAprilTag();
    setDesiredPosition(closestAprilTag.getLeftAlignPose());
  }

  public void rightReefAlign() {
    AprilTag closestAprilTag = getClosestReefAprilTag();
    setDesiredPosition(closestAprilTag.getRightAlignPose());
  }

  /** Returns the closest reef april tag */
  public AprilTag getClosestReefAprilTag() {
    AprilTag[] aprilTags = Constants.reefAprilTags;

    AprilTag closestAprilTag = aprilTags[0];
    double minDistance = currentPosition.getTranslation().getDistance(getAprilTagTranslation2d(closestAprilTag));

    for (int i = 1; i < aprilTags.length; i++) {
      AprilTag aprilTag = aprilTags[i];
      double distance = currentPosition.getTranslation().getDistance(getAprilTagTranslation2d(aprilTag));
      if (distance < minDistance) {
        closestAprilTag = aprilTag;
        minDistance = distance;
      }
    }

    return closestAprilTag;
  }

  public Translation2d getAprilTagTranslation2d(AprilTag tag) {
    return tag.getPose().toPose2d().getTranslation();
  }

  public void setDesiredPosition(Pose2d desiredPosition) {
    this.desiredPosition = desiredPosition;
  }

  public void goToDesiredPosition() {
    double xSpeedMetersPerSecond = translationController.calculate(currentPosition.getX(), desiredPosition.getX());
    double ySpeedMetersPerSecond = translationController.calculate(currentPosition.getY(), desiredPosition.getY());

    xSpeedMetersPerSecond = Math.min(xSpeedMetersPerSecond, SwerveSubsystem.kChosenMaxDriveSpeedMetersPerSecond);
    ySpeedMetersPerSecond = Math.min(ySpeedMetersPerSecond, SwerveSubsystem.kChosenMaxDriveSpeedMetersPerSecond);

    // System.out.println(desiredPosition.getTranslation().minus(currentPosition.getTranslation()) + " " + currentPosition.getY() + " " + xSpeedMetersPerSecond + " " + ySpeedMetersPerSecond);

    double rotationSpeedRadiansPerSecond = rotationController.calculateFromError(SwerveUtils.normalizeAngle(desiredPosition.getRotation().minus(currentPosition.getRotation())).getRadians());
    
    rotationSpeedRadiansPerSecond = Math.min(rotationSpeedRadiansPerSecond, SwerveSubsystem.kPhysicalMaxRotationSpeedRadiansPerSecond);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotationSpeedRadiansPerSecond, currentPosition.getRotation());
    swerveSubsystem.setChassisSpeedsAuto(chassisSpeeds);
  }

  public void stop() {
    swerveSubsystem.stop();
  }

  public boolean atSetpoint() {
    return Math.abs(desiredPosition.getY() - currentPosition.getY()) < 1E-2
    && Math.abs(desiredPosition.getX() - currentPosition.getX()) < 1E-2
    && Math.abs(desiredPosition.getRotation().getRadians() - currentPosition.getRotation().getRadians()) < 1E-2;
  }

  // localization code based on photonvision

  public void calculateCurrentPosition(AprilTag aprilTag) {
    Pose3d cameraPose = VisionConstants.kCameraPose;

    double distance = 0;
    double yaw = 0;

    // calculate based on distance, yaw, and aprilTag


    // set current position to this
    // modify pose in swerve


    // 
  }
}
