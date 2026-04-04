package frc.robot.components.controllers.drive;

import frc.robot.components.controllers.drive.lib.DriveController;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.components.swerve.lib.SwerveConfig;


public class TalonFxDriveController implements DriveController {

    private final Motor motor;
    private final SwerveConfig swerveConfig;
    private final MotorConfig motorConfig;

    private double targetVelocity = 0.0;

    public TalonFxDriveController(SwerveConfig swerveConfigs, MotorConfig motorConfigs) {
        this.swerveConfig = swerveConfigs;
        this.motorConfig = motorConfigs;

        motor = new Kraken(motorConfig);
    }

    public void setVelocity(double metersPerSecond) {
        this.targetVelocity = metersPerSecond;
        motor.setVoltage(this.targetVelocity / swerveConfig.getMaxMetersPerSecond() * swerveConfig.nominalVoltage);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        // return motor.getVelocity();

        // RPS * Meters Per Rotation = Meters Per Second
        return motor.getVelocity() * swerveConfig.driveMetersPerRotation();
    }

    public double getCurrentPosition() {
        return motor.getPosition();
    }

    public int getCanBusId() {
        return motorConfig.canBusId;
    }
}


