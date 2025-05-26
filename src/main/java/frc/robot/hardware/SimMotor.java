package frc.robot.hardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimMotor extends SubsystemBase {
    private double speedRotationsPerSecond = 0;
    private double rotations = 0;

    private double lastUpdateTime = Timer.getTimestamp(); 
    // doesn't work for autonomous swerve for some reason

    // unused
    public void setSpeedRotationsPerSecond(double speed) {
        speedRotationsPerSecond = speed;
    }

    public void setSpeedAndUpdatePosition(double speed) {
        setSpeedRotationsPerSecond(speed);
        if (Timer.getTimestamp() - lastUpdateTime < 0.01) 
            System.out.println(lastUpdateTime + " " + Timer.getTimestamp());
        updatePosition(Timer.getTimestamp() - lastUpdateTime);
    }

    private void updatePosition(double interval) {
        rotations += speedRotationsPerSecond * interval;
        // .out.println(interval);
        lastUpdateTime += interval;
    }

    public double getPositionRotations() {
        return rotations;
    }

    public double getSpeedRotationsPerSecond() {
        return speedRotationsPerSecond;
    }

    public void setPositionRotations(double rotations) {
        this.rotations = rotations;
    }

    // @Override
    // public void periodic() {
    //     double time = getTime();
    //     updatePosition(time - lastUpdateTime);
    //     lastUpdateTime = time;
    // }
}