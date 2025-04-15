package frc.robot.hardware;

public class MotorController {

    public enum MotorConfig {
        // TalonFX
        FrontLeftDrive(0),
        FrontLeftAngle(1),
        FrontRightDrive(2),
        FrontRightAngle(3),
        BackLeftDrive(4),
        BackLeftAngle(5),
        BackRightDrive(6),
        BackRightAngle(7);
        // SparkMax
        // ...

        private int deviceId;

        private MotorConfig(int deviceId) {
            this.deviceId = deviceId;
        }

        public int getDeviceId() {
            return deviceId;
        }
    }

    // public TalonFX constructTalonFX(MotorConfig motorConfig) {
    //     return new TalonFX(motorConfig.deviceId);
    // }
}
