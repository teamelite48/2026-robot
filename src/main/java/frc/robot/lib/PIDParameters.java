package frc.robot.lib;

public class PIDParameters {

    public final double P;
    public final double I;
    public final double D;
    public final double S;
    public final double V;

    public PIDParameters(double p, double i, double d, double s, double v) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.S = s;
        this.V = v;
    }
}