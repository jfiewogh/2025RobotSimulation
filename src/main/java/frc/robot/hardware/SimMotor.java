package frc.robot.hardware;

public class SimMotor {
    private double speedRotationsPerSecond = 0;

    private double rotations = 0;
    
    public SimMotor() {
    }

    public void setSpeedRotationsPerSecond(double speed) {
        speedRotationsPerSecond = speed;
    }

    public void update(double time) {
        rotations += speedRotationsPerSecond * time;
    }

    public double getPositionRotations() {
        return rotations;
    }

    public double getSpeedRotationsPerSecond() {
        return speedRotationsPerSecond;
    }
}