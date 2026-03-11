package frc.robot.components.motors.lib;

public interface Motor {
    public double getPosition();
    public double getVelocity();
    public void setPosition(double position);
    public void setSpeed(double speed);
    public void setVoltage(double d);
    public void setMotionMagicPosition(double position, double feedForward);
    public void stop();
    public void setInitialPosition();
    public void setInitialPosition(double position);
    public void setVelocity(double rpm);
    public void follow(Motor leader, boolean oppose);
}
