package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveUtils {
    public static double getAngularValue(double linearValue, double radius) {
        return linearValue / radius;
    }
    public static double getLinearValue(double angularValue, double radius) {
        return angularValue * radius;
    }
    
    public static Rotation2d normalizeAngle(Rotation2d angle) {
        double degrees = angle.getDegrees();
        while (degrees < -180 || degrees > 180) {
            degrees += degrees < -180 ? 360 : -360;
        }
        return Rotation2d.fromDegrees(degrees);
    }

}