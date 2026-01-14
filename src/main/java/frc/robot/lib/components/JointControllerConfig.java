package frc.robot.lib.components;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.lib.PIDParameters;

public record JointControllerConfig(
    int motorControllerCanId,
    double initDegrees,
    int currentLimit,
    IdleMode initIdleMode,
    boolean isMotorInverted,
    float forwardLimit,
    float reverseLimit,
    double absoluteEncoderPositionConversionFactor,
    double absoluteEncoderOffsetDegrees,
    boolean isAbsoluteEncoderInverted,
    double relativeEncoderPositionConversionFactor,
    PIDParameters pidParams,
    String shuffleboardTab,
    String jointName
) {}