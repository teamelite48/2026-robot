package frc.robot.components.controllers.angle;

import frc.robot.components.controllers.angle.lib.AngleController;
import frc.robot.components.encoders.absolute.CanCoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.NEO;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

import static frc.robot.components.swerve.lib.SwerveMath.*;


public class SparkMaxAngleController implements AngleController {

    private final Motor motor;
    private final CanCoder absoluteEncoder;
    private final MotorConfig motorConfig;
    private final AbsoluteEncoderConfig absoluteEncoderConfig;

    private double targetAngle = 0.0;
    private boolean isInitialized = false;

    public SparkMaxAngleController(MotorConfig motorConfigs, AbsoluteEncoderConfig absEncoderConfig) {

        this.motorConfig = motorConfigs;
        this.absoluteEncoderConfig = absEncoderConfig;

        motor = new NEO(motorConfig);
        absoluteEncoder = new CanCoder(absoluteEncoderConfig);

        motor.setInitialPosition(getAbsoluteAngle());

    }

    // Just about most of the time the motor encoder doesn't initialize properly, so we force it until it do
    public void init() {

        // isInitialized = true;
        
        if (isInitialized) return;

        var absoluteAngle = getAbsoluteAngle();

        motor.setInitialPosition(absoluteAngle);

        this.targetAngle = absoluteAngle;

        setAngle(absoluteAngle);

        isInitialized = true;

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
        return normalizeAngle(absoluteEncoder.getRawPosition() * TAU);
    }

    public boolean isInitialized() {
        return isInitialized;
    }

    public double getRawCurrentAngle() {
        return motor.getPosition();
    }

    public double getTargetAngleWrappedDegrees() {
        return Math.toDegrees(normalizeAngle(targetAngle));
    }

}