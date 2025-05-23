package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mechanism;
import frc.robot.Constants.MechanismSpeed;
import frc.robot.Constants.PIDConstants;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.hardware.SimMotor;

public class SwerveModule {
    private final SimMotor driveSimMotor;
    private final SimMotor angleSimMotor;

    public static final double kPhysicalMaxDriveSpeedMetersPerSecond = MechanismSpeed.kDrive.getMaxMechanismSpeedMetersPerSecond(Constants.kWheelRadius);

    public static final double kMaxAngleSpeedPosition = 0.25; 

    public SwerveModule(MotorConfig driveMotorConfig, MotorConfig angleMotorConfig) {
        driveSimMotor = new SimMotor();
        angleSimMotor = new SimMotor();

        System.out.println(kPhysicalMaxDriveSpeedMetersPerSecond);
    }

    public void setState(SwerveModuleState state) {
        double driveMotorSpeedRotationsPerSecond = Mechanism.kDrive.fromMechanism(Units.radiansToRotations(SwerveUtils.getAngularValue(state.speedMetersPerSecond, Constants.kWheelRadius)));
        
        Rotation2d currentWheelAngle = getWheelAnglePosition();

        Rotation2d error = SwerveUtils.normalizeAngle(state.angle.minus(currentWheelAngle));
        if (Math.abs(error.getDegrees()) > 90) {
            error = SwerveUtils.normalizeAngle(error.plus(Rotation2d.fromDegrees(180)));
            driveMotorSpeedRotationsPerSecond *= -1;
        }

        double angleWheelSpeedRotationsPerSecond = PIDConstants.kModuleAngleController.calculateFromError(error.getRotations());
                
        double angleMotorSpeedRotationsPerSecond = Mechanism.kAngle.fromMechanism(angleWheelSpeedRotationsPerSecond);
        setSimAngleMotorSpeed(angleMotorSpeedRotationsPerSecond);

        setSimDriveMotorSpeed(driveMotorSpeedRotationsPerSecond);
    }

    public SwerveModuleState getModuleState() {
        double speedMetersPerSecond = SwerveUtils.getLinearValue(Units.rotationsToRadians(Mechanism.kDrive.toMechanism(driveSimMotor.getSpeedRotationsPerSecond())), Constants.kWheelRadius);
        return new SwerveModuleState(speedMetersPerSecond, getWheelAnglePosition());
    }

    public void setSimDriveMotorSpeed(double driveMotorSpeedRotationsPerSecond) {
        driveSimMotor.setSpeedAndUpdatePosition(driveMotorSpeedRotationsPerSecond);
    }

    public void setSimAngleMotorSpeed(double angleMotorSpeedRotationsPerSecond) {
        angleSimMotor.setSpeedAndUpdatePosition(angleMotorSpeedRotationsPerSecond);
    }

    public void stop() {
        setSimDriveMotorSpeed(0);
        setSimAngleMotorSpeed(0);
    }

    public double getDriveSimMotorPosition() {
        return driveSimMotor.getPositionRotations();
    }
    public double getAngleSimMotorPosition() {
        return angleSimMotor.getPositionRotations();
    }

    public Rotation2d getWheelAnglePosition() {
        return SwerveUtils.normalizeAngle(Rotation2d.fromRotations(Mechanism.kAngle.toMechanism(getAngleSimMotorPosition())));
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = SwerveUtils.getLinearValue(Mechanism.kDrive.toMechanism(Units.rotationsToRadians(getDriveSimMotorPosition())), Constants.kWheelRadius);
        return new SwerveModulePosition(
            distanceMeters,
            getWheelAnglePosition()
        );
    }
}