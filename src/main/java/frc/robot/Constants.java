// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.hardware.CustomPIDController;
import frc.robot.subsystems.swerve.SwerveUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class RobotConstants {
    public static double kBumperWidthMeters = 1; // random num
    public static double kBumperLengthMeters = 1;
  }

  public static final double kWheelRadius = Units.inchesToMeters(1.5);

  public enum ControllerType {
    KEYBOARD, XBOX;
  }

  public static final ControllerType kControllerType = ControllerType.KEYBOARD;

  public static class PIDConstants {
    // 1 means at the farthest possible point, it is at max speed
    // be careful with D

    public static final CustomPIDController kModuleAngleController = new CustomPIDController(
      1, 0, 0.001, 0.25, MechanismSpeed.kAngle.getMaxMechanismSpeedRotationsPerSecond());

    public static final PIDController kElevatorController = new PIDController(10, 0, 0);
  }

  public static enum Mechanism {
    kDrive(1.0 / 5.0),
    kAngle(1.0 / 3.0 / 4.0),
    kElevator(1.0 / 50.0); // random value

    private final double gearRatio; 

    private Mechanism(double gearRatio) {
      this.gearRatio = gearRatio;
    }

    public double toMechanism(double value) {
      return value * gearRatio;
    }
    public double fromMechanism(double value) {
      return value / gearRatio;
    }
  }

  public static enum MotorSpeed {
    kKraken(6000),
    kVortex(6784);

    private final double freeSpeedRotationsPerMinute;
    private final double freeSpeedRotationsPerSecond;

    private MotorSpeed(double freeSpeedRotationsPerMinute) {
      this.freeSpeedRotationsPerMinute = freeSpeedRotationsPerMinute;
      freeSpeedRotationsPerSecond = freeSpeedRotationsPerMinute / 60;
    }

    public double getFreeSpeedRotationsPerMinute() {
      return freeSpeedRotationsPerMinute;
    }
    public double getFreeSpeedRotationsPerSecond() {
      return freeSpeedRotationsPerSecond;
    }
  }

  public static enum MechanismSpeed {
    kDrive(MotorSpeed.kKraken, Mechanism.kDrive),
    kAngle(MotorSpeed.kKraken, Mechanism.kAngle),
    kElevator(MotorSpeed.kVortex, Mechanism.kElevator);

    private final MotorSpeed motorSpeed;
    private final Mechanism mechanism;

    private MechanismSpeed(MotorSpeed motorSpeed, Mechanism mechanism) {
      this.motorSpeed = motorSpeed;
      this.mechanism = mechanism;
    }

    public double getMaxMechanismSpeedRotationsPerSecond() {
      return mechanism.toMechanism(motorSpeed.getFreeSpeedRotationsPerSecond());
    }
    public double getMaxMechanismSpeedRadiansPerSecond() {
      return Units.rotationsToRadians(getMaxMechanismSpeedRotationsPerSecond());
    }
    public double getMaxMechanismSpeedMetersPerSecond(double radius) {
      return SwerveUtils.getLinearValue(getMaxMechanismSpeedRadiansPerSecond(), radius);
    }
  }

  public class VisionConstants {
    public static final Pose3d kCameraPose = new Pose3d(
      new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)
    );
  }

  public static enum ReefAprilTag {
    // opponent side
    k6(530.49, 130.17, 12.13, 300, 0),
    k7(546.87, 158.50, 12.13, 0, 0),
    k8(530.49, 186.83, 12.13, 60, 0),
    k9(497.77, 186.83, 12.13, 120, 0),
    k10(481.39, 158.50, 12.13, 180, 0),
    k11(497.77, 130.17, 12.13, 240, 0),

    k17(160.39, 130.17, 12.13, 240, 0),
    k18(144.00, 158.50, 12.13, 180, 0),
    k19(160.39, 186.83, 12.13, 120, 0),
    k20(193.10, 186.83, 12.13, 60, 0),
    k21(209.49, 158.50, 12.13, 0, 0),
    k22(193.10, 130.17, 12.13, 300, 0);

    private final Pose3d pose;
    private final Pose2d leftAlignPose;
    private final Pose2d rightAlignPose;

    private ReefAprilTag(double x, double y, double z, double zRotation, double yRotation) {
      pose = new Pose3d(
        Units.inchesToMeters(x), Units.inchesToMeters(y), Units.inchesToMeters(z), 
        new Rotation3d(0, Units.degreesToRadians(yRotation), Units.degreesToRadians(zRotation)));

      // I don't know why these values are flipped
      leftAlignPose = pose.toPose2d().plus(new Transform2d(RobotConstants.kBumperLengthMeters / 2, -0.16, Rotation2d.k180deg));
      rightAlignPose = pose.toPose2d().plus(new Transform2d(RobotConstants.kBumperLengthMeters / 2, 0.16, Rotation2d.k180deg));
    }

    public Pose3d getPose() {
      return pose;
    }

    public Pose2d getLeftAlignPose() {
      return leftAlignPose;
    }
    public Pose2d getRightAlignPose() {
      return rightAlignPose;
    }
  }

  public enum BargeAprilTag {

  }

  public enum SourceAprilTag {

  }

  public enum ProcesserAprilTag {

  }

  public enum AprilTag {
    // values in inches

    k1(657.37, 25.80, 58.50, 126, 0),
    k2(657.37, 291.20, 58.50, 234, 0),
    k12(0, 0, 0, 0, 0),
    k13(0, 0, 0, 0, 0),

    k3(0, 0, 0, 0, 0),
    k16(0, 0, 0, 0, 0),

    k4(0, 0, 0, 0, 30),
    k5(0, 0, 0, 0, 30),
    k14(0, 0, 0, 0, 30),
    k15(0, 0, 0,0, 30);

    private final Pose3d pose;

    private AprilTag(double x, double y, double z, double zRotation, double yRotation) {
      pose = new Pose3d(
        Units.inchesToMeters(x), Units.inchesToMeters(y), Units.inchesToMeters(z), 
        new Rotation3d(0, Units.degreesToRadians(yRotation), Units.degreesToRadians(zRotation)));
    }

    public Pose3d getPose() {
      return pose;
    }
  }
}
