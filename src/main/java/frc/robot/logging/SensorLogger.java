package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

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