package frc.robot.logging;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.lang.reflect.Field;

/**
 * Reflectively finds motor fields on subsystem instances and registers them
 * with RobotContainer.sensorLogger via SensorLogger.addMotor(key, motorInstance).
 *
 * Key format: "/{subsystemFieldName}/{motorFieldName}"
 */
public final class SubsystemMotorAutoRegistrar {

    private SubsystemMotorAutoRegistrar() {}

    public static void register(Object robotContainer) {
        if (robotContainer == null) return;
        if (RobotContainer.sensorLogger == null) return;

        try {
            Field[] rcFields = robotContainer.getClass().getDeclaredFields();
            for (Field rcField : rcFields) {
                rcField.setAccessible(true);
                Object subsystem = rcField.get(robotContainer);
                if (subsystem == null) continue;
                if (!SubsystemBase.class.isInstance(subsystem)) continue;

                String subsystemName = rcField.getName();

                Field[] subFields = subsystem.getClass().getDeclaredFields();
                for (Field sf : subFields) {
                    sf.setAccessible(true);
                    Object candidate = sf.get(subsystem);
                    if (candidate == null) continue;

                    boolean looksLikeMotor = false;

                    // 1) check for your Motor interface/class
                    try {
                        Class<?> motorIfc = Class.forName("frc.robot.components.motors.lib.Motor");
                        if (motorIfc.isInstance(candidate)) {
                            looksLikeMotor = true;
                        }
                    } catch (ClassNotFoundException ignored) {}

                    // 2) heuristic by classname (Kraken, Spark, Talon, Neo, Falcon, CTRE)
                    String cn = candidate.getClass().getSimpleName().toLowerCase();
                    if (cn.contains("kraken") || cn.contains("spark") || cn.contains("talon") ||
                        cn.contains("neo") || cn.contains("falcon") || cn.contains("ctre") || cn.contains("motor")) {
                        looksLikeMotor = true;
                    }

                    if (!looksLikeMotor) continue;

                    try {
                        String key = "/" + subsystemName + "/" + sf.getName();
                        RobotContainer.sensorLogger.addMotor(key, candidate);
                    } catch (Exception ignored) {}
                }
            }
        } catch (Throwable ignored) {}
    }
}