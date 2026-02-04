package frc.robot.components.controllers.angle;

import frc.robot.components.encoders.absolute.CanCoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.Neo550;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

import static frc.robot.components.swerve.lib.SwerveMath.*;


public class SparkMaxAngleController {

    private final Motor motor;
    private final MotorConfig motorConfig;
    // private final RelativeEncoder motorEncoder;
    private final CanCoder absoluteEncoder;
    private final AbsoluteEncoderConfig absoluteEncoderConfig;

    private double targetAngle = 0.0;
    private boolean isInitialized = false;


    public SparkMaxAngleController(MotorConfig motorConfigs, AbsoluteEncoderConfig absEncoderConfig) {

        this.motorConfig = motorConfigs;
        // this.angleConfig = angleControllerConfig;
        this.absoluteEncoderConfig = absEncoderConfig;

        motor = new Neo550(motorConfig);
        absoluteEncoder = new CanCoder(absoluteEncoderConfig);

        motor.setInitialPosition(getAbsoluteAngle());

        // sparkMaxConfig.closedLoop.pid(1.0, 0.0, 0.0);
    }

    // Just about most of the time the motor encoder doesn't initialize properly, so we force it until it do
    public void init() {

        if (isInitialized) return;

        var currentAngle = getCurrentAngle();
        var absoluteAngle = getAbsoluteAngle();

        if (Math.abs(currentAngle - absoluteAngle) < 0.001) {
            isInitialized = true;
            return;
        }

        motor.setInitialPosition(absoluteAngle);
    }

    public void setAngle(double desiredAngle) {

        if (isInitialized == false) return;

        double currentAngle = motor.getPosition();
        var currentAngleMod = normalizeAngle(currentAngle);

        // The target angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedDesiredAngle = desiredAngle + currentAngle - currentAngleMod;

        if (desiredAngle - currentAngleMod > PI) {
            adjustedDesiredAngle -= TAU;
        }
        else if (desiredAngle - currentAngleMod < -PI) {
            adjustedDesiredAngle += TAU;
        }

        this.targetAngle = adjustedDesiredAngle;

        motor.setPosition(this.targetAngle);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return normalizeAngle(motor.getPosition());
    }

    public double getAbsoluteAngle() {
        return normalizeAngle(absoluteEncoder.getPosition());
    }

    public boolean isInitialized() {
        return isInitialized;
    }
}