package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class TurretConfig {

    public static final CANBus CANIVORE = new CANBus("canivore");

    public static final double inputDeadzone = 0.2;

    public static final double motorMaxOutput = 0.35;
    public static final double clockwiseSpeed = motorMaxOutput * 0.25;
    public static final double counterClockwiseSpeed = -clockwiseSpeed;

    public static final double degreesPerMotorRotation = 4;

    public static final double degreesAtLeft = 90;
    public static final double degreesAtCenter = 180;
    public static final double degreesAtRight = 270;

    public static final double encoderLimit = (float) (90 / degreesPerMotorRotation);
    public static final double nominalMotorRotationsPerSecond = 11000 / 60.0;

    public static final double moveWithinDegrees = 3;
    public static final long moveCoolDown = 1000;

    public static final double HOME_POSITION = 0.0;


    public static MotorConfig getMotorConfig() {

        var config = new MotorConfig(16, CANIVORE);

        config.isInverted = true;
        //config.positionConversionFactor = (1.0/90.0) * 360.0;
        config.isBrakeModeEnabled = true;
        config.initialPosition = HOME_POSITION;
        config.pidParameters = new PIDParameters(0.035, 0.0005, 0.0);
        config.forwardLimit = encoderLimit;
        config.reverseLimit = -encoderLimit;

        return config;
    }
}
