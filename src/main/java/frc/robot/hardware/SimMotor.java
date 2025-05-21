package frc.robot.hardware;

import edu.wpi.first.wpilibj.Timer;

public class SimMotor {
    private double speedRotationsPerSecond = 0;
    private double rotations = 0;
    // private double lastUpdateTime = getTime(); // breaks for autonomous

    public void setSpeedRotationsPerSecond(double speed) {
        speedRotationsPerSecond = speed;
        updatePosition(getTime());
    }

    private void updatePosition(double time) {
        rotations += speedRotationsPerSecond * 0.02;
        // lastUpdateTime = time;
    }

    public double getPositionRotations() {
        return rotations;
    }

    public double getSpeedRotationsPerSecond() {
        return speedRotationsPerSecond;
    }

    private static double getTime() {
        return Timer.getTimestamp();
    }
}