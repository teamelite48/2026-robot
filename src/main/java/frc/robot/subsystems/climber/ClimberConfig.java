package frc.robot.subsystems.climber;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ClimberConfig {

    public static final double EXTEND_SPEED = 1.0;
    public static final double RETRACT_SPEED = -1.0;
    public static final double EXTEND_LIMIT = 8.5;  // Home position = 21.5" off the ground; Max legal height=30"
    public static final double HOME_POSITION = 0.0;
    public static final double RETRACT_LIMIT = 0.0;
    public static final double MANUAL_MODIFIER = 0.5;
    public static final double BUFFER_ZONE = 0.75;  // in Inches

    public static final double TARGET_THRESHOLD = 1.0;

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(17);
        // double inchesPerRotation = BUFFER_ZONE * Math.PI;
        double gearRatio = 4.0 * 4.0 * 3.0;  // 48:1

        config.supplyCurrentLimit = 40;
        config.statorCurrentLimit = 80;
        config.isInverted = false;  // Positive for a minion is clockwise
        config.positionConversionFactor = gearRatio; //gearRatio / inchesPerRotation;  // Calculated in inches
        config.isBrakeModeEnabled = true;
        config.initialPosition = HOME_POSITION;
        config.reverseLimit = HOME_POSITION;
        config.forwardLimit = EXTEND_LIMIT;
        config.pidParameters = new PIDParameters(0.7, 0.08, 0.0, 0.0, 0.0);

        return config;
    }
}
