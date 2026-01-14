package frc.robot.controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualShock4Controller {

    public Trigger square;
    public Trigger cross;
    public Trigger circle;
    public Trigger triangle;
    public Trigger l1;
    public Trigger r1;
    public Trigger l2;
    public Trigger r2;
    public Trigger share;
    public Trigger options;
    public Trigger l3;
    public Trigger r3;

    public Trigger ps;
    public Trigger touchpad;

    public Trigger up;
    public Trigger right;
    public Trigger down;
    public Trigger left;


    private GenericHID hid;
    private double deadBand = 0.1;
    private boolean squareInputs = true;

    public DualShock4Controller(int port) {

        hid = new GenericHID(port);

        square = new JoystickButton(hid, 1);
        cross = new JoystickButton(hid, 2);
        circle = new JoystickButton(hid, 3);
        triangle = new JoystickButton(hid, 4);
        l1 = new JoystickButton(hid, 5);
        r1 = new JoystickButton(hid, 6);
        l2 = new JoystickButton(hid, 7);
        r2 = new JoystickButton(hid, 8);
        share = new JoystickButton(hid, 9);
        options = new JoystickButton(hid, 10);
        l3 = new JoystickButton(hid, 11);
        r3 = new JoystickButton(hid, 12);

        ps = new JoystickButton(hid, 13);
        touchpad = new JoystickButton(hid, 14);

        up = new Trigger(() -> hid.getPOV() == 0);
        right = new Trigger(() -> hid.getPOV() == 90);
        down = new Trigger(() -> hid.getPOV() == 180);
        left = new Trigger(() -> hid.getPOV() == 270);
    }

    public Translation2d getLeftAxes() {
        double x = getLeftXAxis();
        double y = getLeftYAxis();

        return normalizeVector(x, y);
    }

    public Translation2d getRightAxes() {
        double x = getRightXAxis();
        double y = getRightYAxis();

        return normalizeVector(x, y);
    }

    public Translation2d normalizeVector(double x, double y) {

        double r = Math.sqrt(x*x + y*y);

        double maxComponent = Math.max(Math.abs(x), Math.abs(y));

        if (maxComponent > 0) {
            x *= r / maxComponent;
            y *= r / maxComponent;
        }

        return new Translation2d(x, y);
    }

    public double getDeadBand() {
        return deadBand;
    }

    public void rumble(double duration) {

        RumbleType rumbleType = RumbleType.kBothRumble;
        double strength = 0.3;

        hid.setRumble(rumbleType, strength);

        var start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < duration) {
            // wait before turning rumble off
        }

        hid.setRumble(rumbleType, 0);
    }

    private double modifyAxis(Double value) {

        if (Math.abs(value) < deadBand) {
            return 0;
        }

        if (squareInputs == true){
            return Math.abs(value) * value;
        }

        return value;
    }

    private double getLeftXAxis() {
        return modifyAxis(hid.getRawAxis(0));
    }
    private double getLeftYAxis() {
        return modifyAxis(hid.getRawAxis(1));
    }
    private double getRightXAxis() {
        return modifyAxis(hid.getRawAxis(2));
    }

    private double getRightYAxis() {
         return modifyAxis(hid.getRawAxis(5));
    }
}
