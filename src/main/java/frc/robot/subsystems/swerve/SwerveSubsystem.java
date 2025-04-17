// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.hardware.MotorController.MotorConfig;

public class SwerveSubsystem extends SubsystemBase {
  private static final double wheelBase = 0.85; // front to back
  private static final double trackWidth = 0.85; // left to right

  private final Translation2d[] locations = new Translation2d[] {
    new Translation2d(wheelBase / 2, trackWidth / 2),
    new Translation2d(wheelBase / 2, -trackWidth / 2),
    new Translation2d(-wheelBase / 2, trackWidth / 2),
    new Translation2d(-wheelBase / 2, -trackWidth / 2)
  };
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations);

  private final SwerveModule frontLeftModule = new SwerveModule(MotorConfig.FrontLeftDrive, MotorConfig.FrontLeftAngle);
  private final SwerveModule frontRightModule = new SwerveModule(MotorConfig.FrontRightDrive, MotorConfig.FrontRightAngle);
  private final SwerveModule backLeftModule = new SwerveModule(MotorConfig.BackLeftDrive, MotorConfig.BackLeftAngle);
  private final SwerveModule backRightModule = new SwerveModule(MotorConfig.BackRightDrive, MotorConfig.BackRightAngle);
  private final SwerveModule[] modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), getModulePositions(), new Pose2d());

  private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  StructArrayPublisher<SwerveModuleState> modulesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  StructPublisher<ChassisSpeeds> currentSpeeds = NetworkTableInstance.getDefault()
    .getStructTopic("CurrentSpeeds", ChassisSpeeds.struct).publish();
  StructPublisher<ChassisSpeeds> otherSpeeds = NetworkTableInstance.getDefault()
    .getStructTopic("OtherSpeeds", ChassisSpeeds.struct).publish();

  StructPublisher<Rotation2d> gyroAngle = NetworkTableInstance.getDefault()
    .getStructTopic("GyroAngle", Rotation2d.struct).publish();


  SwerveModulePosition[] lastPositions;

  public static final double maxDriveSpeedMetersPerSecond = 5;
  // public static final double maxDriveAccelerationMetersPerSecondSquared = 3;

  private static final double driveBaseRadius = Math.hypot(wheelBase / 4, trackWidth / 4);
  private static final double maxRotationSpeedRadiansPerSecond = SwerveModule.getAngularVelocity(SwerveModule.maxDriveSpeedMetersPerSecond, driveBaseRadius);

  private Rotation2d rawGyroRotation = new Rotation2d();

  RobotConfig config;

  Joystick joystick;
  Joystick joystick2;

  Timer timer = new Timer();
  

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(Joystick joystick, Joystick joystick2) {
    lastPositions = getModulePositions();

    System.out.println(maxRotationSpeedRadiansPerSecond);

    initAuto();

    this.joystick = joystick;
    this.joystick2 = joystick2;

    timer.restart();
  }

  public void initAuto() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setChassisSpeedsAuto(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }



  // there is still a small drift



  public void updateOdometer() {
    SwerveModulePosition[] modulePositions = getModulePositions();

    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      moduleDeltas[i] = new SwerveModulePosition(
        modulePositions[i].distanceMeters - lastPositions[i].distanceMeters, 
        modulePositions[i].angle
      );
    }


    Twist2d twist = kinematics.toTwist2d(moduleDeltas);



    rawGyroRotation = Rotation2d.fromRadians(rawGyroRotation.getRadians() + twist.dtheta);



    poseEstimator.update(rawGyroRotation, modulePositions);

    lastPositions = modulePositions;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public Rotation2d getGyroAngle() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = modules[i].getModuleState();
    }
    return moduleStates;
  }

  double previousTime = 0;

  @Override
  public void periodic() {
    // System.out.println("odometer");

    fieldCentricSwerve(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> joystick2.getRawAxis(0));



    // help

    for (int i = 0; i < 4; i++) {
      modules[i].updateSimMotors(timer.get() - previousTime);
    }

    // drifting is worse if after updateOdometer();




    updateOdometer();

    gyroAngle.set(rawGyroRotation);



    posePublisher.set(getPose());

    currentSpeeds.set(kinematics.toChassisSpeeds(getModuleStates()));

    modulesPublisher.set(getModuleStates());


    previousTime = timer.get();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.maxDriveSpeedMetersPerSecond);
    for (int i = 0; i < 4; i++) {
      modules[i].setState(moduleStates[i]);
    }
  }
  public void setChassisSpeedsAuto(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds);
    for (int i = 0; i < 4; i++) {
      modules[i].updateSimMotors(0.02);
    }
    updateOdometer();
  }

  public void fieldCentricSwerve(DoubleSupplier leftStickY, DoubleSupplier leftStickX, DoubleSupplier rightStickX) {
    double xSpeed = -leftStickY.getAsDouble() * maxDriveSpeedMetersPerSecond;
    double ySpeed = -leftStickX.getAsDouble() * maxDriveSpeedMetersPerSecond;
    double rotationSpeed = -rightStickX.getAsDouble() * maxRotationSpeedRadiansPerSecond * 0.5;

    // robotCentricSwerve(xSpeed, ySpeed, rotationSpeed);

    double offsetRadians = -rawGyroRotation.getRadians();


    // xSpeed = xSpeed * Math.cos(offsetRadians) - ySpeed * Math.sin(offsetRadians);
    // ySpeed = xSpeed * Math.sin(offsetRadians) + ySpeed * Math.cos(offsetRadians);

    // ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, rawGyroRotation);

    otherSpeeds.set(speeds);


    setChassisSpeeds(speeds);
  }

  public void robotCentricSwerve(double xSpeed, double ySpeed, double rotationSpeed) {
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    setChassisSpeeds(speeds);
  }

  @Override
  public void simulationPeriodic() {
  }
}
