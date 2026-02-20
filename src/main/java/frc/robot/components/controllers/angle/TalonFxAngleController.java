package frc.robot.components.controllers.angle;

import frc.robot.components.controllers.angle.lib.AngleController;
import frc.robot.components.encoders.absolute.CanCoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

import static frc.robot.components.swerve.lib.SwerveMath.*;


public class TalonFxAngleController implements AngleController {

    private final Motor motor;
    private final CanCoder absoluteEncoder;
    private final MotorConfig motorConfig;
    private final AbsoluteEncoderConfig absoluteEncoderConfig;

    private double targetAngle = 0.0;
    private boolean isInitialized = false;

    public TalonFxAngleController(MotorConfig motorConfigs, AbsoluteEncoderConfig absEncoderConfig) {
        this.motorConfig = motorConfigs;
        this.absoluteEncoderConfig = absEncoderConfig;

        motor = new Kraken(motorConfig);
        absoluteEncoder = new CanCoder(absoluteEncoderConfig);

        motor.setInitialPosition(getAbsoluteAngle());

    }

    // Just about most of the time the motor encoder doesn't initialize properly, so we force it until it do
    public void init() {

        // isInitialized = true;

        if (isInitialized) return;

        // var currentAngle = getCurrentAngle();
        var absoluteAngle = getAbsoluteAngle();

        motor.setInitialPosition(absoluteAngle);

        // Set internal target to match so the motor doesn't jump back to 0
        this.targetAngle = absoluteAngle;

        // setAngle(absoluteAngle);
        isInitialized = true;

        // if (Math.abs(currentAngle - absoluteAngle) < 0.001) {
        // // if (Math.abs(currentAngle - absoluteAngle) < 0.1) {
        //     isInitialized = true;
        //     return;
        // }
    }

    public void setAngle(double desiredAngle) {
        this.targetAngle = desiredAngle;
        motor.setPosition(desiredAngle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return normalizeAngle(motor.getPosition());
    }

    public double getAbsoluteAngle() {
        // rawPosition is [0, 1), multiplying by TAU makes it [0, 2pi)
        // normalizeAngle ensures it stays strictly in [0, 2pi)
        // return normalizeAngle(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * TAU);
        // return normalizeAngle(absoluteEncoder.getPosition());
        return normalizeAngle(absoluteEncoder.getRawPosition() * TAU);
    }

    public boolean isInitialized() {
        return isInitialized;
    }
}