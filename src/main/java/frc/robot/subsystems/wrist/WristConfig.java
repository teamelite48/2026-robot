package frc.robot.subsystems.wrist;

import frc.robot.RobotConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class WristConfig {

    public static final double POSITION_CONVERSION_FACTOR = (1.0/13.5) * 360.0;
    public static final PIDParameters PID_PARAMETERS = new PIDParameters(0.02, 0.0, 0.0);
    public static final double MANUAL_TARGET_MODIFIER = 1.0;
    public static final double MAX_SPEED_CORAL = 0.25;
    public static final double MAX_SPEED_ALGAE = 0.15;
    public static final double TARGET_THRESHOLD = 1.0;
    public static final boolean IS_BRAKE_MODE_ENABLED = true;
    public static final double MAX_VELOCITY = 1000;

    public static final double INITIAL_DEGREES = 0.0;
    public static final double HOME_TILT_DEGREES = 90.0;

    public static final  MotorConfig getRightMotorConfig() {
        var config = new MotorConfig(14, RobotConfig.CANIVORE_48);

        config.isInverted = false;
        config.initialPosition = INITIAL_DEGREES;
        config.positionConversionFactor = POSITION_CONVERSION_FACTOR;

        config.isBrakeModeEnabled = IS_BRAKE_MODE_ENABLED;

        return config;
    }

    public static final  MotorConfig getLeftMotorConfig() {
        var config = new MotorConfig(15, RobotConfig.CANIVORE_48);

        config.isInverted = true;
        config.initialPosition = INITIAL_DEGREES;
        config.positionConversionFactor = POSITION_CONVERSION_FACTOR;
        config.isBrakeModeEnabled = IS_BRAKE_MODE_ENABLED;

        return config;
    }
}
