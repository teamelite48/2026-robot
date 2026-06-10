package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.geometry.Pose2d;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/*
Reusable SensorLogger with motor auto-adapters and per-entry sampling & delta thresholds.

Usage:
  // default (every loop, no delta threshold)
  sensorLogger.addDouble("/some/key", () -> sensor.getDoubleValue());

  // sample at 10 Hz and only write when value changes by >= 0.1
  sensorLogger.addDouble("/some/key", () -> sensor.getDoubleValue(), 10.0, 0.1);

  // addMotor remains the same (auto-detects velocity/position methods)
*/
public class SensorLogger {

    private static final class Entry {
        final DoubleSupplier supplier;
        final Object reflectionEntry;
        final Method appendMethod;
        final String key;

        // sampling/delta
        long lastAppendTimeNanos = 0;
        double lastValue = Double.NaN;
        final long periodNanos; // 0 == every loop
        final double deltaThreshold; // >=0

        Entry(DoubleSupplier supplier, Object reflectionEntry, Method appendMethod, String key,
              long periodNanos, double deltaThreshold) {
            this.supplier = supplier;
            this.reflectionEntry = reflectionEntry;
            this.appendMethod = appendMethod;
            this.key = key;
            this.periodNanos = periodNanos;
            this.deltaThreshold = (deltaThreshold >= 0.0) ? deltaThreshold : 0.0;
        }

        void tryAppend(long now, double v) {
            // sampling
            if (periodNanos > 0 && (now - lastAppendTimeNanos) < periodNanos) return;
            // delta threshold
            if (!Double.isNaN(lastValue) && Math.abs(v - lastValue) < deltaThreshold) {
                return;
            }
            // write
            if (reflectionEntry != null && appendMethod != null) {
                try {
                    appendMethod.invoke(reflectionEntry, v);
                } catch (Exception ignored) {}
            } else {
                SmartDashboard.putNumber(key, v);
            }
            lastValue = v;
            lastAppendTimeNanos = now;
        }
    }

    private final List<Entry> entries = new ArrayList<>();
    private final Object dataLogInstance;
    private final Constructor<?> doubleCtor;
    private final Method doubleAppend;

    public SensorLogger() {
        Object logInst = null;
        Constructor<?> dCtor = null;
        Method dAppend = null;
        try {
            Class<?> dataLogMgrClass = Class.forName("edu.wpi.first.util.datalog.DataLogManager");
            Class<?> dataLogClass = Class.forName("edu.wpi.first.util.datalog.DataLog");
            Class<?> doubleLogEntryClass = Class.forName("edu.wpi.first.util.datalog.DoubleLogEntry");

            Method startMethod = dataLogMgrClass.getMethod("start");
            startMethod.invoke(null);

            Method getLogMethod = dataLogMgrClass.getMethod("getLog");
            logInst = getLogMethod.invoke(null);

            dCtor = doubleLogEntryClass.getConstructor(dataLogClass, String.class);
            dAppend = doubleLogEntryClass.getMethod("append", double.class);
        } catch (Exception ignored) {
            // datalog not available -> fallback to SmartDashboard
        }
        dataLogInstance = logInst;
        doubleCtor = dCtor;
        doubleAppend = dAppend;
    }

    // Default: every loop, no delta threshold
    public void addDouble(String key, DoubleSupplier supplier) {
        addDouble(key, supplier, 0.0, 0.0);
    }

    // sampleHz: 0.0 => every loop. deltaThreshold: >=0.0 to enable change-only logging.
    public void addDouble(String key, DoubleSupplier supplier, double sampleHz, double deltaThreshold) {
        Object entryObj = null;
        Method append = null;
        if (dataLogInstance != null && doubleCtor != null && doubleAppend != null) {
            try {
                entryObj = doubleCtor.newInstance(dataLogInstance, key);
                append = doubleAppend;
            } catch (Exception ignored) {
                entryObj = null;
                append = null;
            }
        }
        long periodNanos = 0;
        if (sampleHz > 0.0) {
            periodNanos = (long)(1_000_000_000.0 / sampleHz);
        }
        entries.add(new Entry(supplier, entryObj, append, key, periodNanos, deltaThreshold));
    }

    // existing motor autodetect remains unchanged (kept below from prior version)
    public void addMotor(String keyPrefix, Object motor) {
        if (motor == null) return;

        // 1) Try motor.getEncoder().getVelocity()/getPosition() (SparkMAX)
        try {
            Method getEncoder = motor.getClass().getMethod("getEncoder");
            Object encoder = getEncoder.invoke(motor);
            if (encoder != null) {
                Method getVelocity = findMethod(encoder.getClass(), "getVelocity", "getRPM", "getRate");
                Method getPosition = findMethod(encoder.getClass(), "getPosition", "getPosition360", "getRotations");
                if (getVelocity != null) {
                    Method gv = getVelocity;
                    addDouble(keyPrefix + "/velocity", () -> {
                        try {
                            Object v = gv.invoke(encoder);
                            return toDouble(v);
                        } catch (Exception ex) {
                            return Double.NaN;
                        }
                    });
                }
                if (getPosition != null) {
                    Method gp = getPosition;
                    addDouble(keyPrefix + "/position", () -> {
                        try {
                            Object v = gp.invoke(encoder);
                            return toDouble(v);
                        } catch (Exception ex) {
                            return Double.NaN;
                        }
                    });
                }
                return;
            }
        } catch (NoSuchMethodException ignored) {
        } catch (Exception ignored) {
        }

        // 2) CTRE style
        try {
            Method vel = findMethod(motor.getClass(), "getSelectedSensorVelocity", "getSelectedSensorVelocity0");
            Method pos = findMethod(motor.getClass(), "getSelectedSensorPosition", "getSelectedSensorPosition0");
            if (vel != null) {
                Method vmethod = vel;
                addDouble(keyPrefix + "/velocity", () -> {
                    try {
                        Object o = vmethod.invoke(motor);
                        return toDouble(o);
                    } catch (Exception e) {
                        return Double.NaN;
                    }
                });
            }
            if (pos != null) {
                Method pmethod = pos;
                addDouble(keyPrefix + "/position", () -> {
                    try {
                        Object o = pmethod.invoke(motor);
                        return toDouble(o);
                    } catch (Exception e) {
                        return Double.NaN;
                    }
                });
            }
            if (vel != null || pos != null) return;
        } catch (Exception ignored) {
        }

        // 3) direct getVelocity/getPosition on motor
        try {
            Method vel = findMethod(motor.getClass(), "getVelocity", "getRPM", "getRate");
            Method pos = findMethod(motor.getClass(), "getPosition", "getSelectedSensorPosition", "getEncoderPosition");
            if (vel != null) {
                Method vmethod = vel;
                addDouble(keyPrefix + "/velocity", () -> {
                    try {
                        Object o = vmethod.invoke(motor);
                        return toDouble(o);
                    } catch (Exception e) {
                        return Double.NaN;
                    }
                });
            }
            if (pos != null) {
                Method pmethod = pos;
                addDouble(keyPrefix + "/position", () -> {
                    try {
                        Object o = pmethod.invoke(motor);
                        return toDouble(o);
                    } catch (Exception e) {
                        return Double.NaN;
                    }
                });
            }
            if (vel != null || pos != null) return;
        } catch (Exception ignored) {
        }

        // 4) last resort
        try {
            Method anyVel = findMethod(motor.getClass(), "getAppliedOutput", "get");
            if (anyVel != null) {
                Method vmethod = anyVel;
                addDouble(keyPrefix + "/velocity", () -> {
                    try {
                        Object o = vmethod.invoke(motor);
                        return toDouble(o);
                    } catch (Exception e) {
                        return Double.NaN;
                    }
                });
                return;
            }
        } catch (Exception ignored) {
        }
    }

    // Public helper: register an external encoder object under keyPrefix (e.g. "/turret/absolute")
    public void addEncoder(String keyPrefix, Object encoder) {
        if (encoder == null) return;

        // try to discover position method
        Method posMethod = findMethod(encoder.getClass(),
            "getAbsolutePosition",
            "getAbsoluteAngle",
            "getPosition",
            "getDistance",
            "get"
        );

        // try to discover velocity/rate method
        Method velMethod = findMethod(encoder.getClass(),
            "getVelocity",
            "getRate",
            "getSpeed"
        );

        if (posMethod != null) {
            Method pm = posMethod;
            // default: 50 Hz, small delta threshold to avoid noise
            addDouble(keyPrefix + "/position", () -> {
                try {
                    Object o = pm.invoke(encoder);
                    return toDouble(o);
                } catch (Exception e) {
                    return Double.NaN;
                }
            }, 50.0, 0.01);
        }

        if (velMethod != null) {
            Method vm = velMethod;
            addDouble(keyPrefix + "/velocity", () -> {
                try {
                    Object o = vm.invoke(encoder);
                    return toDouble(o);
                } catch (Exception e) {
                    return Double.NaN;
                }
            }, 50.0, 0.1);
        }
    }

    // Register an IMU/gyro-like object under keyPrefix (e.g. "/drive/pigeon2")
    public void addGyro(String keyPrefix, Object gyro) {
        if (gyro == null) return;

        // possible method names for heading/yaw
        Method yawMethod = findMethod(gyro.getClass(),
                "getYaw", "getAngle", "getHeading", "getFusedHeading", "getYawDegrees", "getYawPitchRoll");
        // pitch/roll
        Method pitchMethod = findMethod(gyro.getClass(), "getPitch", "getPitchDegrees");
        Method rollMethod = findMethod(gyro.getClass(), "getRoll", "getRollDegrees");
        // some APIs return a Rotation2d or a container - handle via reflection below
        Method rotation2dMethod = findMethod(gyro.getClass(), "getRotation2d", "getRotation");
        // rate / angular velocity
        Method rateMethod = findMethod(gyro.getClass(),
                "getRate", "getVelocity", "getAngularVelocity", "getRateZ", "getGyroRate");

        // helper to read possibly-wrapped Rotation2d-like objects
        java.util.function.DoubleSupplier yawSupplier = null;
        if (yawMethod != null) {
            Method ym = yawMethod;
            yawSupplier = () -> {
                try {
                    Object o = ym.invoke(gyro);
                    if (o == null) return Double.NaN;
                    // Rotation2d-like -> try .getDegrees()/.getRadians()/.getDegrees
                    Method m = findMethod(o.getClass(), "getDegrees", "getDegrees()");
                    if (m != null) return toDouble(m.invoke(o));
                    Method m2 = findMethod(o.getClass(), "getRadians");
                    if (m2 != null) return Math.toDegrees(toDouble(m2.invoke(o)));
                    return toDouble(o);
                } catch (Exception ex) {
                    return Double.NaN;
                }
            };
        } else if (rotation2dMethod != null) {
            Method rm = rotation2dMethod;
            yawSupplier = () -> {
                try {
                    Object o = rm.invoke(gyro);
                    if (o == null) return Double.NaN;
                    Method getDeg = findMethod(o.getClass(), "getDegrees", "getDegrees()");
                    if (getDeg != null) return toDouble(getDeg.invoke(o));
                    Method getRad = findMethod(o.getClass(), "getRadians");
                    if (getRad != null) return Math.toDegrees(toDouble(getRad.invoke(o)));
                    return toDouble(o);
                } catch (Exception ex) {
                    return Double.NaN;
                }
            };
        }

        java.util.function.DoubleSupplier pitchSupplier = null;
        if (pitchMethod != null) {
            Method pm = pitchMethod;
            pitchSupplier = () -> {
                try { return toDouble(pm.invoke(gyro)); } catch (Exception e) { return Double.NaN; }
            };
        }

        java.util.function.DoubleSupplier rollSupplier = null;
        if (rollMethod != null) {
            Method rm2 = rollMethod;
            rollSupplier = () -> {
                try { return toDouble(rm2.invoke(gyro)); } catch (Exception e) { return Double.NaN; }
            };
        }

        java.util.function.DoubleSupplier rateSupplier = null;
        if (rateMethod != null) {
            Method rmm = rateMethod;
            rateSupplier = () -> {
                try { return toDouble(rmm.invoke(gyro)); } catch (Exception e) { return Double.NaN; }
            };
        }

        // register discovered values with sensible defaults (50 Hz, small delta)
        if (yawSupplier != null) addDouble(keyPrefix + "/yaw_deg", yawSupplier, 50.0, 0.01);
        if (pitchSupplier != null) addDouble(keyPrefix + "/pitch_deg", pitchSupplier, 50.0, 0.01);
        if (rollSupplier != null) addDouble(keyPrefix + "/roll_deg", rollSupplier, 50.0, 0.01);
        if (rateSupplier != null) addDouble(keyPrefix + "/rate_dps", rateSupplier, 50.0, 0.1);
    }

    // Register limelights found via NetworkTables and log their botpose arrays (x,y,z)
    public void addLimelightNetworkTables() {
        try {
            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            // don't call getTableNames() (not available on this WPILib); use common limelight names
            String[] candidates = new String[] { "limelight", "limelight1", "limelight2", "limelight-front", "limelight-rear" };

            for (String name : candidates) {
                if (name == null) continue;
                String lower = name.toLowerCase();
                if (!lower.startsWith("limelight")) continue;
                NetworkTable t = nt.getTable(name);
                if (t == null) continue;

                final NetworkTable table = t;
                final String base = "/" + name;

                // suppliers read current NT entries each call
                addDouble(base + "/pose/x", () -> {
                    double[] a = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
                    if (a.length >= 3) return a[0];
                    a = table.getEntry("botpose").getDoubleArray(new double[0]);
                    return (a.length >= 3) ? a[0] : Double.NaN;
                }, 20.0, 0.001);

                addDouble(base + "/pose/y", () -> {
                    double[] a = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
                    if (a.length >= 3) return a[1];
                    a = table.getEntry("botpose").getDoubleArray(new double[0]);
                    return (a.length >= 3) ? a[1] : Double.NaN;
                }, 20.0, 0.001);

                addDouble(base + "/pose/z", () -> {
                    double[] a = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
                    if (a.length >= 3) return a[2];
                    a = table.getEntry("botpose").getDoubleArray(new double[0]);
                    return (a.length >= 3) ? a[2] : Double.NaN;
                }, 20.0, 0.001);

                // scan table keys for std/dev/confidence-like entries and log them (scalar or array)
                try {
                    java.util.Set<String> keys = table.getKeys();
                    for (String k : keys) {
                        if (k == null) continue;
                        String kl = k.toLowerCase();
                        if (!(kl.contains("std") || kl.contains("sigma") || kl.contains("confid"))) continue;

                        final String entryKey = k;
                        // try array first
                        try {
                            double[] arr = table.getEntry(entryKey).getDoubleArray(new double[0]);
                            if (arr != null && arr.length > 1) {
                                for (int i = 0; i < arr.length; ++i) {
                                    final int idx = i;
                                    String dest = base + "/" + entryKey + "_" + idx;
                                    addDouble(dest, () -> {
                                        double[] a = table.getEntry(entryKey).getDoubleArray(new double[0]);
                                        return (a.length > idx) ? a[idx] : Double.NaN;
                                    }, 20.0, 0.0001);
                                }
                                continue;
                            }
                        } catch (Throwable ignored) {}

                        // fallback to scalar
                        try {
                            String dest = base + "/" + entryKey;
                            addDouble(dest, () -> table.getEntry(entryKey).getDouble(Double.NaN), 20.0, 0.0001);
                        } catch (Throwable ignored) {}
                    }
                } catch (Throwable ignored) {}
            }
        } catch (Throwable ignored) {}
    }

    // Register a Pose2d supplier and log x,y and rotation (deg)
    public void addPoseSupplier(String keyPrefix, Supplier<Pose2d> poseSupplier, double sampleHz, double deltaThreshold) {
        if (poseSupplier == null) return;
        addDouble(keyPrefix + "/x", () -> {
            try {
                Pose2d p = poseSupplier.get();
                return (p == null) ? Double.NaN : p.getX();
            } catch (Throwable t) { return Double.NaN; }
        }, sampleHz, deltaThreshold);

        addDouble(keyPrefix + "/y", () -> {
            try {
                Pose2d p = poseSupplier.get();
                return (p == null) ? Double.NaN : p.getY();
            } catch (Throwable t) { return Double.NaN; }
        }, sampleHz, deltaThreshold);

        addDouble(keyPrefix + "/rotation_deg", () -> {
            try {
                Pose2d p = poseSupplier.get();
                if (p == null) return Double.NaN;
                return Math.toDegrees(p.getRotation().getRadians());
            } catch (Throwable t) { return Double.NaN; }
        }, sampleHz, deltaThreshold);
    }

    // Auto-discover pose-providing methods across subsystems in RobotContainer
    public void addRobotPoseAuto(Object robotContainer) {
        if (robotContainer == null) return;
        try {
            java.lang.reflect.Field[] fields = robotContainer.getClass().getDeclaredFields();
            for (java.lang.reflect.Field f : fields) {
                f.setAccessible(true);
                Object subsystem = f.get(robotContainer);
                if (subsystem == null) continue;
                // try methods on the subsystem that return Pose2d
                String[] methodNames = new String[] { "getPose", "getEstimatedPose", "getPoseMeters", "getOdometryPose", "getCurrentPose" };
                for (String mName : methodNames) {
                    try {
                        Method m = subsystem.getClass().getMethod(mName);
                        if (m != null && Pose2d.class.isAssignableFrom(m.getReturnType())) {
                            Supplier<Pose2d> sup = () -> {
                                try { return (Pose2d) m.invoke(subsystem); } catch (Throwable t) { return null; }
                            };
                            String key = "/" + f.getName() + "/pose";
                            addPoseSupplier(key, sup, 50.0, 0.01);
                            // prefer first found pose method on this subsystem
                            break;
                        }
                    } catch (NoSuchMethodException ignored) {}
                }
            }
            // if none found, try a top-level robot pose method on robotContainer
            try {
                Method m = robotContainer.getClass().getMethod("getRobotPose");
                if (m != null && Pose2d.class.isAssignableFrom(m.getReturnType())) {
                    Supplier<Pose2d> sup = () -> {
                        try { return (Pose2d) m.invoke(robotContainer); } catch (Throwable t) { return null; }
                    };
                    addPoseSupplier("/robot/pose", sup, 50.0, 0.01);
                }
            } catch (NoSuchMethodException ignored) {}
        } catch (Throwable ignored) {}
    }

    public void periodic() {
        long now = System.nanoTime();
        for (Entry e : entries) {
            try {
                double v = e.supplier.getAsDouble();
                e.tryAppend(now, v);
            } catch (Exception ignored) {}
        }
    }

    private static Method findMethod(Class<?> cls, String... names) {
        for (String n : names) {
            try {
                Method m = cls.getMethod(n);
                if (m != null) return m;
            } catch (NoSuchMethodException ignored) {}
        }
        return null;
    }

    private static double toDouble(Object o) {
        if (o == null) return Double.NaN;
        if (o instanceof Number) return ((Number) o).doubleValue();
        try { return Double.parseDouble(o.toString()); } catch (Exception ignored) { return Double.NaN; }
    }
}