package frc.robot.logging;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.lang.reflect.Field;
import java.util.HashSet;
import java.util.Set;

/**
 * Reflectively finds motor fields on subsystem instances and registers them
 * with RobotContainer.sensorLogger via SensorLogger.addMotor(key, motorInstance).
 *
 * Key format: "/{subsystemFieldName}/{friendlyName}"
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
                // track used friendly names for this subsystem to avoid collisions
                Set<String> used = new HashSet<>();

                for (Field sf : subFields) {
                    sf.setAccessible(true);
                    Object candidate = sf.get(subsystem);
                    if (candidate == null) continue;

                    boolean looksLikeMotor = false;
                    boolean looksLikeEncoder = false;
                    boolean looksLikeGyro = false;

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

                    // 3) encoder heuristics (class name contains "encoder", "cancoder", "absolute", "dutycycle", "analog")
                    if (cn.contains("encoder") || cn.contains("cancoder") || cn.contains("absolute") || cn.contains("dutycycle") || cn.contains("analog")) {
                        looksLikeEncoder = true;
                    }

                    // 4) gyro/IMU heuristics (Pigeon2/NavX/IMU)
                    if (cn.contains("pigeon") || cn.contains("pigeon2") || cn.contains("navx") ||
                        cn.contains("gyro") || cn.contains("imu") || cn.contains("ahrs")) {
                        looksLikeGyro = true;
                    }

                    // compute friendly name
                    String raw = sf.getName();
                    String friendly = makeFriendlyName(raw);

                    // try to discover a numeric device id on the motor and append it to the friendly name
                    String idSuffix = "";
                    try {
                        Integer dev = discoverDeviceId(candidate);
                        if (dev != null) idSuffix = "_" + dev;
                    } catch (Throwable ignored) {}

                    // ensure uniqueness
                    String baseFriendly = friendly + idSuffix;
                    String unique = baseFriendly;
                    int suffix = 1;
                    while (used.contains(unique)) {
                        unique = baseFriendly + "_" + suffix++;
                    }
                    used.add(unique);

                    // register motors
                    if (looksLikeMotor) {
                        try {
                            String key = "/" + subsystemName + "/" + unique;
                            RobotContainer.sensorLogger.addMotor(key, candidate);
                        } catch (Exception ignored) {}
                    }

                    // register external encoders
                    if (looksLikeEncoder) {
                        try {
                            String key = "/" + subsystemName + "/" + unique;
                            RobotContainer.sensorLogger.addEncoder(key, candidate);
                        } catch (Exception ignored) {}
                    }

                    // register gyros / IMUs
                    if (looksLikeGyro) {
                        try {
                            String key = "/" + subsystemName + "/" + unique;
                            RobotContainer.sensorLogger.addGyro(key, candidate);
                        } catch (Exception ignored) {}
                    }
                }
            }
        } catch (Throwable ignored) {}
    }

    // Produce a nicer name from a field name:
    // - camelCase -> snake_case
    // - strip common suffixes like _motor, _controller, _master, _slave
    // - preserve direction tokens (left/right)
    private static String makeFriendlyName(String fieldName) {
        if (fieldName == null || fieldName.isEmpty()) return fieldName;

        // camelCase -> snake_case
        String snake = fieldName.replaceAll("([a-z0-9])([A-Z])", "$1_$2").toLowerCase();

        // remove common suffixes but keep left/right
        String[] removeTokens = new String[] { "_motor", "motor", "_controller", "controller", "_master", "master", "_slave", "slave", "_mc", "_m" };
        for (String t : removeTokens) {
            if (snake.endsWith(t)) {
                snake = snake.substring(0, snake.length() - t.length());
                break;
            }
            // also remove embedded tokens
            snake = snake.replace(t, "");
        }

        // collapse multiple underscores and trim
        snake = snake.replaceAll("_+", "_");
        if (snake.startsWith("_")) snake = snake.substring(1);
        if (snake.endsWith("_")) snake = snake.substring(0, snake.length() - 1);

        // fallback to original if empty
        if (snake.isEmpty()) return fieldName.toLowerCase();
        return snake;
    }

    // Try to find a numeric device id on common motor objects (CAN id, device id, etc.)
    private static Integer discoverDeviceId(Object obj) {
        if (obj == null) return null;
        String[] methodNames = new String[] {
            "getDeviceID", "getDeviceNumber", "getDeviceId", "getCANId", "getCANID", "getId", "getPort"
        };
        for (String m : methodNames) {
            try {
                java.lang.reflect.Method mm = obj.getClass().getMethod(m);
                Object res = mm.invoke(obj);
                if (res instanceof Number) return ((Number) res).intValue();
                if (res instanceof String) {
                    try { return Integer.parseInt((String) res); } catch (Exception ignored) {}
                }
            } catch (NoSuchMethodException ignored) {}
            catch (Throwable ignored) {}
        }
        // try common field names
        String[] fieldNames = new String[] { "deviceID", "deviceNumber", "canId", "id", "port" };
        for (String f : fieldNames) {
            try {
                java.lang.reflect.Field ff = obj.getClass().getDeclaredField(f);
                ff.setAccessible(true);
                Object val = ff.get(obj);
                if (val instanceof Number) return ((Number) val).intValue();
                if (val instanceof String) {
                    try { return Integer.parseInt((String) val); } catch (Exception ignored) {}
                }
            } catch (NoSuchFieldException ignored) {}
            catch (Throwable ignored) {}
        }
        return null;
    }
}