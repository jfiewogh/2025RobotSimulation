package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CustomPIDController;
import frc.robot.hardware.SimMotor;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TestSubsystem extends SubsystemBase {
    private final SimMotor armMotor = new SimMotor();
    private final SimMotor intakeMotor = new SimMotor();

    private final CustomPIDController armController = new CustomPIDController(1, 0, 0.001, 0.25, 1);
    private final CustomPIDController intakeController = new CustomPIDController(1, 0, 0, 0.25, 2);

    private final Pose3d initialPose0 = new Pose3d(
        new Translation3d(0, 0, 0.61 + 0.04), 
        new Rotation3d(
            Units.degreesToRadians(0), 
            Units.degreesToRadians(-89.5), 
            Units.degreesToRadians(0)
        )
    );
    private final Pose3d initialPose1 = new Pose3d(
        new Translation3d(0.01, 0, 1.248 + 0.04),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(90),
            Units.degreesToRadians(0)
        )
    );
    private final Pose3d initialPose2 = new Pose3d(
        new Translation3d(0.0015, 0, 0.4415 + 0.04),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(180),
            Units.degreesToRadians(0)
        )
    ); 
    private final Pose3d initialPose3 = new Pose3d(
        new Translation3d(0.28, 0, 0.155 + 0.04),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(-90),
            Units.degreesToRadians(0)
        )
    ); 

    private Pose3d pose0 = new Pose3d(initialPose0.getTranslation(), initialPose0.getRotation());
    private Pose3d pose1 = new Pose3d(initialPose1.getTranslation(), initialPose1.getRotation());
    private Pose3d pose2 = new Pose3d(initialPose2.getTranslation(), initialPose2.getRotation());
    private Pose3d pose3 = new Pose3d(initialPose3.getTranslation(), initialPose3.getRotation());

    private Pose3d conePose = Pose3d.kZero;

    private final double pose1To2Distance = initialPose2.getZ() - initialPose1.getZ();

    private final StructPublisher<Pose3d> component0 = NetworkTableInstance.getDefault()
        .getStructTopic("Component0", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> component1 = NetworkTableInstance.getDefault()
        .getStructTopic("Component1", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> component2 = NetworkTableInstance.getDefault()
        .getStructTopic("Component2", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> component3 = NetworkTableInstance.getDefault()
        .getStructTopic("Component3", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> conePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Cone", Pose3d.struct).publish();


    private Rotation2d armDesiredAngle = Rotation2d.kZero;
    private Rotation2d intakeDesiredAngle = Rotation2d.kZero;

    private final SwerveSubsystem swerveSubsystem;

    public TestSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public void setArmDesiredAngle(Rotation2d angle) {
        armDesiredAngle = angle;
    }
    public Command setArmDesiredAngleCommand(Rotation2d angle) {
        return new InstantCommand(() -> setArmDesiredAngle(angle));
    }

    public void setIntakeDesiredAngle(Rotation2d angle) {
        intakeDesiredAngle = angle;
    }
    public Command setIntakeDesiredAngleCommand(Rotation2d angle) {
        return new InstantCommand(() -> setIntakeDesiredAngle(angle));
    }


    @Override
    public void periodic() {

        // ARM
        double armMotorSpeed = armController.calculateFromSetpoint(armMotor.getPositionRotations(), armDesiredAngle.getRotations());
        armMotor.setSpeedRotationsPerSecond(armMotorSpeed);

        double armPitchRadians = Units.rotationsToRadians(armMotor.getPositionRotations());

        pose1 = new Pose3d(pose1.getTranslation(), new Rotation3d(0, armPitchRadians + initialPose1.getRotation().getY(), 0));

        double x = pose1To2Distance * Math.sin(armPitchRadians);
        double z = initialPose1.getZ() + pose1To2Distance * Math.cos(armPitchRadians);
        pose2 = new Pose3d(x, 0, z, new Rotation3d(0, armPitchRadians + Math.PI, 0));


        // INTAKE
        double intakeMotorSpeed = intakeController.calculateFromSetpoint(intakeMotor.getPositionRotations(), intakeDesiredAngle.getRotations());
        intakeMotor.setSpeedRotationsPerSecond(intakeMotorSpeed);

        double intakePitchRadians = Units.rotationsToRadians(intakeMotor.getPositionRotations());
        pose3 = new Pose3d(pose3.getTranslation(), new Rotation3d(0, intakePitchRadians + initialPose3.getRotation().getY(), 0));


        conePose = new Pose3d(swerveSubsystem.getPose()).plus(new Transform3d(
            pose2.getTranslation().plus(new Translation3d(0.15, pose2.getRotation())).plus(new Translation3d(-0.2, pose2.getRotation().plus(new Rotation3d(0, Math.PI / 2, 0)))), 
            pose2.getRotation().plus(new Rotation3d(0, Math.PI, 0))));


        // simulation
        component0.set(pose0);
        component1.set(pose1);
        component2.set(pose2);
        component3.set(pose3);
        conePublisher.set(conePose);
    }
}
