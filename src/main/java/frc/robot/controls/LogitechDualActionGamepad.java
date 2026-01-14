package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechDualActionGamepad {

    public Trigger x;
    public Trigger a;
    public Trigger b;
    public Trigger y;
    public Trigger lb;
    public Trigger rb;
    public Trigger lt;
    public Trigger rt;
    public Trigger back;
    public Trigger start;
    public Trigger l3;
    public Trigger r3;

    public Trigger up;
    public Trigger right;
    public Trigger down;
    public Trigger left;


    private GenericHID hid;
    private double deadBand = 0.01;
    private boolean squareInputs = true;

    public LogitechDualActionGamepad(int port) {

        hid = new GenericHID(port);

        x = new JoystickButton(hid, 1);
        a = new JoystickButton(hid, 2);
        b = new JoystickButton(hid, 3);
        y = new JoystickButton(hid, 4);
        lb = new JoystickButton(hid, 5);
        rb = new JoystickButton(hid, 6);
        lt = new JoystickButton(hid, 7);
        rt = new JoystickButton(hid, 8);
        back = new JoystickButton(hid, 9);
        start = new JoystickButton(hid, 10);
        l3 = new JoystickButton(hid, 11);
        r3 = new JoystickButton(hid, 12);

        up = new Trigger(() -> hid.getPOV() == 0);
        right = new Trigger(() -> hid.getPOV() == 90);
        down = new Trigger(() -> hid.getPOV() == 180);
        left = new Trigger(() -> hid.getPOV() == 270);
    }

    public double getLeftXAxis() {
        return modifyAxis(hid.getRawAxis(0));
    }
    public double getLeftYAxis() {
        return modifyAxis(hid.getRawAxis(1));
    }
    public double getRightXAxis() {
        return modifyAxis(hid.getRawAxis(2));
    }
    public double getRightYAxis() {
        return modifyAxis(hid.getRawAxis(3));
    }

    private double modifyAxis(Double value) {

        if (Math.abs(value) < deadBand) {
            return 0;
        }

        if (squareInputs == true){
            return Math.abs(value) * Math.abs(value) * value;
        }

        return value;
    }
}
