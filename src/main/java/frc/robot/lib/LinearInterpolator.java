package frc.robot.lib;

import java.util.HashMap;

public class LinearInterpolator {

    HashMap<Integer, Double> domainAndRange;

    public LinearInterpolator(HashMap<Integer, Double> domainAndRange) {
        this.domainAndRange = domainAndRange;
    }

    public double calculate(double x) {

        int x1 = (int) Math.floor(x);
        int x2 = (int) Math.ceil(x);
        double y1 = domainAndRange.getOrDefault(x1, 27.0);
        double y2 = domainAndRange.getOrDefault(x2, 27.0);

        double y = (y2 - y1) * (x - x1) + y1;

        return y;
    }
}
