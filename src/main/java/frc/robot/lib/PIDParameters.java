package frc.robot.lib;

public class PIDParameters {

    public final double P;
    public final double I;
    public final double D;

    public PIDParameters(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
    }
}