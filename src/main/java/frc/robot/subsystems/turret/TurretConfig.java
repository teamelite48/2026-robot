package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConfig;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class TurretConfig {

    public static final double TURRET_ENCODER_OFFSET_DEGREES = 109.95;  //126.9


    public static final double inputDeadzone = 0.2;
    public static final double FEED_FORWARD_VOLTS = 2.0;
    public static final double DEGREES_TOLERANCE = 2.5;
    public static final double TURRET_INIT_DEGREES = 7.0;
    public static final double TURRET_GEAR_RATIO = 9.0;
    public static final double MANUAL_DEADBAND = 0.05;

    public static final double motorMaxOutput = 0.5;   //0.2 
    public static final double clockwiseSpeed = motorMaxOutput; // motorMaxOutput * 0.25;
    public static final double counterClockwiseSpeed = -clockwiseSpeed;

    public static final double CCW_LIMIT = -270.0;
    public static final double CW_LIMIT = 270.0;

    public static final double degreesPerMotorRotation = 40.0;

    public static final double HOME_POSITION = 0.0;
    public static final double degreesAtCenter = 100.0;

    public static final double motorForwardLimit = CW_LIMIT / 360.0; //(float) (90 / degreesPerMotorRotation);
    public static final double motorBackwardsLimit = CCW_LIMIT / 360.0; //(float) (90 / degreesPerMotorRotation);
    public static final double nominalMotorRotationsPerSecond = 7200.0 / 60.0;

    public static final double CCW_SOFT_MOVEMENT_LIMIT = -70.0;
    public static final double CW_SOFT_MOVEMENT_LIMIT = 0.0;

    public static final double BACKWARDS_BIAS_MODIFIER = 1.0825;

    public static final double moveWithinDegrees = 3;
    public static final long moveCoolDown = 1000;

    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.625594, 4.034536); 
    public static final Translation2d RED_HUB_LOCATION = new Translation2d(11.915394, 4.034536);

    public static final Translation2d BLUE_LEFT_PASS_AREA = new Translation2d(2.001647, 6.051804);
    public static final Translation2d BLUE_RIGHT_PASS_AREA = new Translation2d(2.001647, 2.017268);
    public static final Translation2d RED_LEFT_PASS_AREA = new Translation2d(14.539341, 2.017268);
    public static final Translation2d RED_RIGHT_PASS_AREA = new Translation2d(14.539341, 6.051804);
        

    public static final double BLUE_TRENCH_LINE = 4.333294;
    public static final double RED_TRENCH_LINE = 12.887694;
    public static final double Y_CENTER_LINE = 4.034536;

    //TODO: Find home/starting position of turret for start of match configuration

    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(16, RobotConfig.CANIVORE_48);

        config.isInverted = false;   //Motor flipped to add 2:1 reduction - originally true
        config.positionConversionFactor = (18.0 / 1.0); // ADDED 2:1 REDUCTION // (1.0/90.0) * 360.0;
        config.isBrakeModeEnabled = true;
        config.initialPosition = HOME_POSITION;
        config.pidParameters = new PIDParameters(5.0, 0.0, 0.1, 0.5, 0.12); // 25.0, 0.0, 1.7, 0.25, 0.12
        config.enableFOC = false;
        config.forwardLimit = motorForwardLimit;
        config.reverseLimit = motorBackwardsLimit;
        config.supplyCurrentLimit = 40;  //40
        config.statorCurrentLimit = 100;  //100
        config.motionMagicCruiseVelocity = 1.5; //2.5  // rotations/sec  (smooth start)
        config.motionMagicAcceleration = 2.5; //6.0   // rotations/sec^2 (smooth stop)
        config.motionMagicJerk = 100.0; //100.0           // change if overshooting

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
