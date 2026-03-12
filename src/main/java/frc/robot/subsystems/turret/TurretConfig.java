package frc.robot.subsystems.turret;

import frc.robot.RobotConfig;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class TurretConfig {

    public static final double TURRET_ENCODER_OFFSET_DEGREES = 126.9;

    public static final double inputDeadzone = 0.2;
    public static final double FEED_FORWARD_VOLTS = 2.0;
    public static final double DEGREES_TOLERANCE = 2.0;

    public static final double motorMaxOutput = 0.2;
    public static final double clockwiseSpeed = motorMaxOutput; // motorMaxOutput * 0.25;
    public static final double counterClockwiseSpeed = -clockwiseSpeed;

    public static final double CCW_LIMIT = -90.0;
    public static final double CW_LIMIT = 180.0;

    public static final double degreesPerMotorRotation = 40.0;

    public static final double HOME_POSITION = 0.0;
    public static final double degreesAtLeft = 0.0;
    public static final double degreesAtCenter = 90.0;
    public static final double degreesAtRight = 180.0;

    public static final double motorForwardLimit = CW_LIMIT / 360.0; //(float) (90 / degreesPerMotorRotation);
    public static final double motorBackwardsLimit = CCW_LIMIT / 360.0; //(float) (90 / degreesPerMotorRotation);
    public static final double nominalMotorRotationsPerSecond = 7200.0 / 60.0;

    public static final double moveWithinDegrees = 3;
    public static final long moveCoolDown = 1000;

    //TODO: Find home/starting position of turret for start of match configuration

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(16, RobotConfig.CANIVORE_48);

        config.isInverted = true;
        config.positionConversionFactor = (9.0 / 1.0); // (1.0/90.0) * 360.0;
        config.isBrakeModeEnabled = true;
        config.initialPosition = HOME_POSITION;
        config.pidParameters = new PIDParameters(70.0, 0.0, 2.5, 0.7, 0.12); // 25.0, 0.0, 0.0, 2.0, 0.12
        config.enableFOC = false;
        config.forwardLimit = motorForwardLimit;
        config.reverseLimit = motorBackwardsLimit;
        config.supplyCurrentLimit = 40;
        config.statorCurrentLimit = 100;
        config.motionMagicCruiseVelocity = 2.5; //2.5  // rotations/sec  (smooth start)
        config.motionMagicAcceleration = 2.5; //6.0   // rotations/sec^2 (smooth stop)
        config.motionMagicJerk = 40.0; //40.0           // change if overshooting

        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigTurret() {
        var config = new AbsoluteEncoderConfig(10, RobotConfig.CANIVORE_48);
        config.positionConversionFactor = 360.0;
        config.offset = TURRET_ENCODER_OFFSET_DEGREES;
        config.isInverted = false;
        return config;
    }
}
