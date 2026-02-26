package frc.robot.subsystems.climber;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ClimberConfig {

    public static final double EXTEND_SPEED = 0.5;
    public static final double RETRACT_SPEED = -0.50;
    public static final double EXTEND_LIMIT = 2.0;
    public static final double HOME_POSITION = 0.0;
    public static final double RETRACT_LIMIT = 0.0;
    public static final double MANUAL_MODIFIER = 0.5;
    public static final double BUFFER_ZONE = 1.5;  // in Inches

    public static final double TARGET_THRESHOLD = 1.0;

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(17);
        double inchesPerRotation = BUFFER_ZONE * Math.PI;
        double gearRatio = 4.0 * 4.0 * 3.0;  // 48:1

        config.currentLimit = 60;
        config.isInverted = true;
        config.positionConversionFactor = gearRatio / inchesPerRotation; //1.0;  // Calculated in inches
        config.isBrakeModeEnabled = true;
        config.initialPosition = HOME_POSITION;
        config.reverseLimit = HOME_POSITION;
        config.forwardLimit = EXTEND_LIMIT;
        config.pidParameters = new PIDParameters(0.4, 0.0, 0.0);

        return config;
    }
}
