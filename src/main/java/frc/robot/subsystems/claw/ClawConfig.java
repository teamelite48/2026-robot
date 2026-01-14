package frc.robot.subsystems.claw;

import frc.robot.RobotConfig;
import frc.robot.components.motors.lib.MotorConfig;

public class ClawConfig {

    public static final double ALGAE_INTAKE_SPEED = 0.55;
    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double CORAL_OUTTAKE_SPEED = -0.1;
    public static final double ALGAE_OUTAKE_SPEED = -0.6;
    public static final double HOLD_SPEED = 0.15;

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(11, RobotConfig.CANIVORE_48);

        config.isInverted = false;
        config.isBrakeModeEnabled = true;

        return config;
    }
}
