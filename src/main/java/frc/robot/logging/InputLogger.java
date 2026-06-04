package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.DualShock4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;

/*
Reflection-based InputLogger: uses edu.wpi.first.util.datalog at runtime if available,
otherwise falls back to SmartDashboard. This compiles without requiring datalog on the classpath.
*/
public class InputLogger {

    private final DualShock4Controller pilot;
    private final DualShock4Controller copilot;
    private final DualShock4Controller testController;

    private final LogEntry[] axisEntries;
    private final BoolLogEntry[] buttonEntries;
    private final boolean[] lastButtonValues;

    private static final String[] BUTTON_NAMES = new String[] {
            "square","cross","circle","triangle",
            "l1","r1","l2","r2",
            "l3","r3","options",
            "up","right","down","left"
    };

    public InputLogger(DualShock4Controller pilot, DualShock4Controller copilot, DualShock4Controller testController) {
        this.pilot = pilot;
        this.copilot = copilot;
        this.testController = testController;

        String[] axisKeys = new String[] {
            "/controllers/pilot/left_x",
            "/controllers/pilot/left_y",
            "/controllers/pilot/right_x",
            "/controllers/pilot/right_y",

            "/controllers/copilot/left_x",
            "/controllers/copilot/left_y",
            "/controllers/copilot/right_x",
            "/controllers/copilot/right_y",

            "/controllers/test/left_x",
            "/controllers/test/left_y",
            "/controllers/test/right_x",
            "/controllers/test/right_y"
        };

        axisEntries = new LogEntry[axisKeys.length];
        // try to bind DoubleLogEntry via reflection
        try {
            Class<?> dataLogMgrClass = Class.forName("edu.wpi.first.util.datalog.DataLogManager");
            Class<?> dataLogClass = Class.forName("edu.wpi.first.util.datalog.DataLog");
            Class<?> doubleLogEntryClass = Class.forName("edu.wpi.first.util.datalog.DoubleLogEntry");

            Method startMethod = dataLogMgrClass.getMethod("start");
            startMethod.invoke(null);

            Method getLogMethod = dataLogMgrClass.getMethod("getLog");
            Object dataLogInstance = getLogMethod.invoke(null);

            Constructor<?> ctor = doubleLogEntryClass.getConstructor(dataLogClass, String.class);
            Method appendMethod = doubleLogEntryClass.getMethod("append", double.class);

            for (int i = 0; i < axisKeys.length; ++i) {
                Object entryObj = ctor.newInstance(dataLogInstance, axisKeys[i]);
                axisEntries[i] = new ReflectionEntry(entryObj, appendMethod);
            }
        } catch (Exception e) {
            for (int i = 0; i < axisKeys.length; ++i) {
                axisEntries[i] = new SmartDashboardEntry(axisKeys[i]);
            }
        }

        int buttonsPerController = BUTTON_NAMES.length;
        int totalButtons = buttonsPerController * 3;
        buttonEntries = new BoolLogEntry[totalButtons];
        lastButtonValues = new boolean[totalButtons];

        String[] controllerPrefixes = new String[] { "/controllers/pilot/buttons/", "/controllers/copilot/buttons/", "/controllers/test/buttons/" };

        try {
            Class<?> dataLogMgrClass = Class.forName("edu.wpi.first.util.datalog.DataLogManager");
            Class<?> dataLogClass = Class.forName("edu.wpi.first.util.datalog.DataLog");
            Class<?> boolLogEntryClass = Class.forName("edu.wpi.first.util.datalog.BooleanLogEntry");

            Method startMethod = dataLogMgrClass.getMethod("start");
            startMethod.invoke(null);

            Method getLogMethod = dataLogMgrClass.getMethod("getLog");
            Object dataLogInstance = getLogMethod.invoke(null);

            Constructor<?> boolCtor = boolLogEntryClass.getConstructor(dataLogClass, String.class);
            Method boolAppend = boolLogEntryClass.getMethod("append", boolean.class);

            for (int c = 0; c < 3; ++c) {
                for (int b = 0; b < buttonsPerController; ++b) {
                    String key = controllerPrefixes[c] + BUTTON_NAMES[b];
                    Object entryObj = boolCtor.newInstance(dataLogInstance, key);
                    buttonEntries[c * buttonsPerController + b] = new ReflectionBoolEntry(entryObj, boolAppend);
                    lastButtonValues[c * buttonsPerController + b] = false;
                }
            }
        } catch (Exception e) {
            for (int c = 0; c < 3; ++c) {
                for (int b = 0; b < BUTTON_NAMES.length; ++b) {
                    String key = controllerPrefixes[c] + BUTTON_NAMES[b];
                    buttonEntries[c * BUTTON_NAMES.length + b] = new SmartDashboardBoolEntry(key);
                    lastButtonValues[c * BUTTON_NAMES.length + b] = false;
                }
            }
        }
    }

    public void periodic() {
        safeAppend(0, pilot, Axis.LEFT_X);
        safeAppend(1, pilot, Axis.LEFT_Y);
        safeAppend(2, pilot, Axis.RIGHT_X);
        safeAppend(3, pilot, Axis.RIGHT_Y);

        safeAppend(4, copilot, Axis.LEFT_X);
        safeAppend(5, copilot, Axis.LEFT_Y);
        safeAppend(6, copilot, Axis.RIGHT_X);
        safeAppend(7, copilot, Axis.RIGHT_Y);

        safeAppend(8, testController, Axis.LEFT_X);
        safeAppend(9, testController, Axis.LEFT_Y);
        safeAppend(10, testController, Axis.RIGHT_X);
        safeAppend(11, testController, Axis.RIGHT_Y);

        logButtonsForController(0, pilot);
        logButtonsForController(1, copilot);
        logButtonsForController(2, testController);
    }

    private void logButtonsForController(int controllerIndex, DualShock4Controller ctrl) {
        if (ctrl == null) return;
        int base = controllerIndex * BUTTON_NAMES.length;
        try {
            boolean[] values = new boolean[] {
                    safeGetTrigger(ctrl.square),
                    safeGetTrigger(ctrl.cross),
                    safeGetTrigger(ctrl.circle),
                    safeGetTrigger(ctrl.triangle),
                    safeGetTrigger(ctrl.l1),
                    safeGetTrigger(ctrl.r1),
                    safeGetTrigger(ctrl.l2),
                    safeGetTrigger(ctrl.r2),
                    safeGetTrigger(ctrl.l3),
                    safeGetTrigger(ctrl.r3),
                    safeGetTrigger(ctrl.options),
                    safeGetTrigger(ctrl.up),
                    safeGetTrigger(ctrl.right),
                    safeGetTrigger(ctrl.down),
                    safeGetTrigger(ctrl.left)
            };

            for (int i = 0; i < values.length; ++i) {
                boolean v = values[i];
                int idx = base + i;
                if (v != lastButtonValues[idx]) {
                    if (buttonEntries[idx] != null) {
                        buttonEntries[idx].append(v);
                    } else {
                        SmartDashboard.putBoolean("/controllers/" + (controllerIndex==0?"pilot":controllerIndex==1?"copilot":"test") + "/buttons/" + BUTTON_NAMES[i], v);
                    }
                    lastButtonValues[idx] = v;
                }
            }
        } catch (Exception ignored) {}
    }

    private boolean safeGetTrigger(Trigger t) {
        if (t == null) return false;
        try {
            return t.getAsBoolean();
        } catch (Throwable ignored) {
            try {
                Method m = Trigger.class.getMethod("get");
                Object res = m.invoke(t);
                if (res instanceof Boolean) return (Boolean) res;
            } catch (Exception ignored2) {}
        }
        return false;
    }

    private void safeAppend(int index, DualShock4Controller ctrl, Axis axis) {
        if (ctrl == null) return;
        try {
            var l = ctrl.getLeftAxes();
            var r = ctrl.getRightAxes();
            double v;
            switch (axis) {
                case LEFT_X: v = l.getX(); break;
                case LEFT_Y: v = l.getY(); break;
                case RIGHT_X: v = r.getX(); break;
                case RIGHT_Y: v = r.getY(); break;
                default: v = 0.0;
            }
            if (axisEntries[index] != null) {
                axisEntries[index].append(v);
            } else {
                String keys[] = {
                    "/controllers/pilot/left_x","/controllers/pilot/left_y","/controllers/pilot/right_x","/controllers/pilot/right_y",
                    "/controllers/copilot/left_x","/controllers/copilot/left_y","/controllers/copilot/right_x","/controllers/copilot/right_y",
                    "/controllers/test/left_x","/controllers/test/left_y","/controllers/test/right_x","/controllers/test/right_y"
                };
                SmartDashboard.putNumber(keys[index], v);
            }
        } catch (Exception ignored) {}
    }

    private enum Axis { LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y }

    private interface LogEntry { void append(double v); }

    private static class ReflectionEntry implements LogEntry {
        private final Object entryInstance;
        private final Method appendMethod;
        ReflectionEntry(Object entryInstance, Method appendMethod) {
            this.entryInstance = entryInstance;
            this.appendMethod = appendMethod;
        }
        @Override public void append(double v) {
            try { appendMethod.invoke(entryInstance, v); } catch (Exception ignored) {}
        }
    }

    private static class SmartDashboardEntry implements LogEntry {
        private final String key;
        SmartDashboardEntry(String key) { this.key = key; }
        @Override public void append(double v) { SmartDashboard.putNumber(key, v); }
    }

    private interface BoolLogEntry { void append(boolean v); }

    private static class ReflectionBoolEntry implements BoolLogEntry {
        private final Object entryInstance;
        private final Method appendMethod;
        ReflectionBoolEntry(Object entryInstance, Method appendMethod) {
            this.entryInstance = entryInstance;
            this.appendMethod = appendMethod;
        }
        @Override public void append(boolean v) {
            try { appendMethod.invoke(entryInstance, v); } catch (Exception ignored) {}
        }
    }

    private static class SmartDashboardBoolEntry implements BoolLogEntry {
        private final String key;
        SmartDashboardBoolEntry(String key) { this.key = key; }
        @Override public void append(boolean v) { SmartDashboard.putBoolean(key, v); }
    }
}