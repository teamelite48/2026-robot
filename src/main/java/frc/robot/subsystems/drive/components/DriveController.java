package frc.robot.subsystems.drive.components;

import static frc.robot.subsystems.drive.DriveConfig.*;

import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

public class DriveController {

    private final Motor motor;

    private double targetVelocity = 0.0;

    public DriveController(int id) {

        var config = new MotorConfig(id);

        config.isInverted = IS_INVERTED;
        config.isBrakeModeEnabled = true;
        config.positionConversionFactor = DRIVE_POSITION_TO_METERS_CONVERSION_FACTOR;
        config.currentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
        config.initialPosition = 0.0;

        motor = new Kraken(config);
    }

    public void setVelocity(double metersPerSecond) {
        this.targetVelocity = metersPerSecond;
        motor.setVoltage(this.targetVelocity / MAX_METERS_PER_SECOND * NOMINAL_VOLTAGE);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }

    public double getCurrentPosition() {
        return motor.getPosition();
    }
}


