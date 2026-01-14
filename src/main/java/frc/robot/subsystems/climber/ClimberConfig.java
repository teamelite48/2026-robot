package frc.robot.subsystems.climber;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ClimberConfig {

    public static final double EXTEND_SPEED = 1.0;
    public static final double RETRACT_SPEED = -0.50;
    public static final double EXTEND_LIMIT = 1225.244;
    public static final double HOME_POSITION = 0.0;
    public static final double RETRACT_LIMIT = -48000; //-150.0;

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(17);

        config.isInverted = true;
        config.positionConversionFactor = (1.0/40.0) * 360.0;
        config.isBrakeModeEnabled = true;
        config.initialPosition = HOME_POSITION;
        config.pidParameters = new PIDParameters(0.4, 0.0, 0.0);

        return config;
    }
}
