package frc.robot.util;

public class CoolDownTimer {

    long coolDownMillis;
    long lastStartMillis = System.currentTimeMillis();
    private boolean isCool = true;

    public CoolDownTimer(long coolDownMillis) {
        this.coolDownMillis = coolDownMillis;
    }

    public void start() {
        isCool = false;
        lastStartMillis = System.currentTimeMillis();
    }

    public boolean isCool() {
        if (isCool == true) {
            return true;
        }

        long currentMillis = System.currentTimeMillis();

        if (currentMillis - lastStartMillis >= coolDownMillis) {
            isCool = true;
        }

        return isCool;
    }
}