package frc.robot.hardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimMotor extends SubsystemBase {
    private double speedRotationsPerSecond = 0;
    private double rotations = 0;
    private double lastUpdateTime = getTime();

    public void setSpeedRotationsPerSecond(double speed) {
        speedRotationsPerSecond = speed;
    }

    public void setSpeedAndUpdatePosition(double speed) {
        setSpeedRotationsPerSecond(speed);
        updatePosition(0.02);
    }

    private void updatePosition(double interval) {
        rotations += speedRotationsPerSecond * interval;
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

    public void setPositionRotations(double rotations) {
        this.rotations = rotations;
    }

    @Override
    public void periodic() {
        double time = getTime();
        updatePosition(time - lastUpdateTime);
        lastUpdateTime = time;
    }
}