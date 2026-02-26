package frc.robot.components.motors.lib;

public interface Motor {
    public double getPosition();
    public double getVelocity();
    // public int getCanId();
    public void setPosition(double position);
    public void setSpeed(double speed);
    public void setVoltage(double d);
    public void stop();
    public void setInitialPosition();
    public void setInitialPosition(double position);
}
