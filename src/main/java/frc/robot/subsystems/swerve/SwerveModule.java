package frc.robot.subsystems.swerve;

import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.hardware.SimMotor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {
    private final SimMotor driveSimMotor;
    private final SimMotor angleSimMotor;

    private static final double driveGearRatio = 1.0 / 5.0;
    private static final double angleGearRatio = 1.0 / 3.0 / 4.0;
    public static final double wheelRadius = Units.inchesToMeters(1.5);

    public static final double maxDriveSpeedMetersPerSecond = 5;

    private static final PIDController pidController = new PIDController(50, 1, 10);
    

    public SwerveModule(MotorConfig driveMotorConfig, MotorConfig angleMotorConfig) {
        driveSimMotor = new SimMotor();
        angleSimMotor = new SimMotor();
    }

    public void setState(SwerveModuleState state) {
        double driveMotorSpeedRotationsPerSecond = driveWheelToMotor(Units.radiansToRotations(getAngularVelocity(state.speedMetersPerSecond, wheelRadius)));
        
        Rotation2d currentAngleWheelPosition = getWheelAnglePosition(); // this is wrong
        
        Rotation2d error = normalizeAngle(state.angle.minus(currentAngleWheelPosition));
        // FUTURE: optimize the error
        if (Math.abs(error.getDegrees()) > 90) {
            error = normalizeAngle(error.plus(Rotation2d.fromDegrees(180)));
            driveMotorSpeedRotationsPerSecond *= -1;
        }

        double angleWheelSpeed = pidController.getP() * error.getRotations();

        double angleMotorSpeedRotationsPerSecond = angleWheelToMotor(angleWheelSpeed);
        
        setSimAngleMotorSpeed(angleMotorSpeedRotationsPerSecond);

        setSimDriveMotorSpeed(driveMotorSpeedRotationsPerSecond);
    }

    public SwerveModuleState getModuleState() {
        double speedMetersPerSecond = getLinearVelocity(Units.rotationsToRadians(driveMotorToWheel(driveSimMotor.getSpeedRotationsPerSecond())), wheelRadius);
        return new SwerveModuleState(speedMetersPerSecond, getWheelAnglePosition());
    }

    public void setSimDriveMotorSpeed(double driveMotorSpeedRotationsPerSecond) {
        driveSimMotor.setSpeedRotationsPerSecond(driveMotorSpeedRotationsPerSecond);
    }

    public void setSimAngleMotorSpeed(double angleMotorSpeedRotationsPerSecond) {
        angleSimMotor.setSpeedRotationsPerSecond(angleMotorSpeedRotationsPerSecond);
    }

    public void updateSimMotors(double time) {
        driveSimMotor.update(time);
        angleSimMotor.update(time);
    }

    public double getDriveSimMotorPosition() {
        return driveSimMotor.getPositionRotations();
    }
    public double getAngleSimMotorPosition() {
        return angleSimMotor.getPositionRotations();
    }

    public Rotation2d getWheelAnglePosition() {
        // System.out.println(getAngleRotorPosition());
        // System.out.println(Rotation2d.fromRotations(angleMotorToWheel(getAngleRotorPosition())));
        return normalizeAngle(Rotation2d.fromRotations(angleMotorToWheel(getAngleSimMotorPosition())));
    }

    public static double driveMotorToWheel(double value) {
        return value * driveGearRatio;
    }
    public static double driveWheelToMotor(double value) {
        return value / driveGearRatio;
    }
    public static double angleMotorToWheel(double value) {
        return value * angleGearRatio;
    }
    public static double angleWheelToMotor(double value) {
        return value / angleGearRatio;
    }

    public static Rotation2d normalizeAngle(Rotation2d angle) {
        double degrees = angle.getDegrees();
        while (degrees < -180 || degrees > 180) {
            degrees += degrees < -180 ? 360 : -360;
        }
        return Rotation2d.fromDegrees(degrees);
    }

    public static double getAngularVelocity(double linearVelocity, double radius) {
        return linearVelocity / radius;
    }
    public static double getLinearVelocity(double angularVelocity, double radius) {
        return angularVelocity * radius;
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = driveMotorToWheel(Units.rotationsToRadians(getDriveSimMotorPosition())) * wheelRadius;
        return new SwerveModulePosition(
            distanceMeters,
            getWheelAnglePosition()
        );
    }
}