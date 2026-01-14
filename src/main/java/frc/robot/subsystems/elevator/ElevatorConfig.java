package frc.robot.subsystems.elevator;

import frc.robot.RobotConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ElevatorConfig {

    public static final double INITIAL_POSITION = 0.0;
    public static final double HOME_POSITIION = INITIAL_POSITION + 0.2;
    public static final double EXTEND_SPEED = 0.85;
    public static final double RETRACT_SPEED = -0.3;
    public static final PIDParameters PID_PARAMETERS = new PIDParameters(0.2, 0.0, 0.0);
    public static final double MANUAL_TARGET_MODIFIER = 0.25;

    public static final double TARGET_THRESHOLD = 0.5;
    public static final double MAX_HEIGHT = 48.0;

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(16, RobotConfig.CANIVORE_48);

        config.positionConversionFactor = 1.0;
        config.forwardLimit = MAX_HEIGHT;
        config.reverseLimit = 0.0;
        config.isInverted = true;
        config.initialPosition = INITIAL_POSITION;
        config.isBrakeModeEnabled = true;

        return config;
    }
}
