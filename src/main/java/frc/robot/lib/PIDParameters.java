package frc.robot.lib;

public class PIDParameters {

    public final double P;
    public final double I;
    public final double D;
    public Double S = null;
    public Double V = null;

    public PIDParameters(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.S = 0.0;
        this.V = 0.0;
    }

    public PIDParameters(double p, double i, double d, double s, double v) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.S = s;
        this.V = v;
    }
}