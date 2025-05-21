package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CustomPIDController;
import frc.robot.hardware.SimMotor;

public class TestSubsystem extends SubsystemBase {
    private final SimMotor motor = new SimMotor();

    private Rotation2d desiredAngle = Rotation2d.kZero;

    private CustomPIDController angleController = new CustomPIDController(0.8, 0, 0, 0.25, 2);

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

    private double pose1To2Distance = initialPose2.getZ() - initialPose1.getZ();


    private final StructPublisher<Pose3d> component0 = NetworkTableInstance.getDefault()
        .getStructTopic("Component0", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> component1 = NetworkTableInstance.getDefault()
        .getStructTopic("Component1", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> component2 = NetworkTableInstance.getDefault()
        .getStructTopic("Component2", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> component3 = NetworkTableInstance.getDefault()
        .getStructTopic("Component3", Pose3d.struct).publish();


    public void setDesiredAngle(Rotation2d angle) {
        desiredAngle = angle;
    }

    public Command setDesiredAngleCommand(Rotation2d angle) {
        return new InstantCommand(() -> setDesiredAngle(angle));
    }


    @Override
    public void periodic() {
        double speed = angleController.calculateFromSetpoint(motor.getPositionRotations(), desiredAngle.getRotations());
        
        System.out.println(desiredAngle.getRotations() + " " + motor.getPositionRotations());
        motor.setSpeedRotationsPerSecond(speed);

        double pose1PitchRadians = Units.rotationsToRadians(-motor.getPositionRotations());

        pose1 = new Pose3d(pose1.getTranslation(), new Rotation3d(0, pose1PitchRadians, 0));

        // rotation matrix
        double pose2Thing = pose1PitchRadians;

        pose2 = new Pose3d(pose2.getTranslation(), new Rotation3d(0, pose1PitchRadians + Math.PI / 2, 0));


        component0.set(pose0);
        component1.set(pose1);
        component2.set(pose2);
        component3.set(pose3);
    }
}
