// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deploy;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class DeployConfig {

    public static final double EXTEND_SPEED = 0.4;
    public static final double RETRACT_SPEED = -0.4;
    public static final double EXTEND_LIMIT = 4500.0;
    public static final double MID_POSITION = 0.0;
    public static final double RETRACT_LIMIT = 0.0;
    public static final double HOME_POSITION = RETRACT_LIMIT + 750.0;

    public static MotorConfig getIntakeDeployConfig() {

        var config = new MotorConfig(13);

        config.isInverted = false;
        config.forwardLimit = EXTEND_LIMIT;
        config.reverseLimit = RETRACT_LIMIT;
        config.positionConversionFactor = (30.0 / 16.0) * 27.0 * (54.0 / 28.0); //97.634 - Changed 9.0 to 27.0 to reflect the new MAXPlanetary reduction
        config.initialPosition = RETRACT_LIMIT;
        config.pidParameters = new PIDParameters(0.001, 0.0, 0.0); //0.5, 0.01, 0.0
        config.isBrakeModeEnabled = false;
        config.currentLimit = 60;

        return config;
    }
}
