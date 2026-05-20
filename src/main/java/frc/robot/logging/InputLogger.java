package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.DualShock4Controller;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;

/*
Replaced implementation to avoid compile-time dependency on edu.wpi.first.util.datalog.
It will use datalog via reflection if available, otherwise fallback to SmartDashboard.
*/
public class InputLogger {

    private final DualShock4Controller pilot;
    private final DualShock4Controller copilot;
    private final DualShock4Controller testController;

    // simple entries that either append to a DoubleLogEntry (via reflection) or write to SmartDashboard
    private final LogEntry[] entries;

    // entry index order:
    // 0 pilot_left_x, 1 pilot_left_y, 2 pilot_right_x, 3 pilot_right_y
    // 4 copilot_left_x, 5 copilot_left_y, 6 copilot_right_x, 7 copilot_right_y
    // 8 test_left_x, 9 test_left_y, 10 test_right_x, 11 test_right_y

    public InputLogger(DualShock4Controller pilot, DualShock4Controller copilot, DualShock4Controller testController) {
        this.pilot = pilot;
        this.copilot = copilot;
        this.testController = testController;

        String[] keys = new String[] {
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

        LogEntry[] temp = new LogEntry[keys.length];

        // Try to wire to edu.wpi.first.util.datalog.DoubleLogEntry via reflection.
        try {
            Class<?> dataLogMgrClass = Class.forName("edu.wpi.first.util.datalog.DataLogManager");
            Class<?> dataLogClass = Class.forName("edu.wpi.first.util.datalog.DataLog");
            Class<?> doubleLogEntryClass = Class.forName("edu.wpi.first.util.datalog.DoubleLogEntry");

            // call DataLogManager.start();
            Method startMethod = dataLogMgrClass.getMethod("start");
            startMethod.invoke(null);

            // get DataLogManager.getLog();
            Method getLogMethod = dataLogMgrClass.getMethod("getLog");
            Object dataLogInstance = getLogMethod.invoke(null);

            // constructor DoubleLogEntry(DataLog log, String path)
            Constructor<?> ctor = doubleLogEntryClass.getConstructor(dataLogClass, String.class);
            Method appendMethod = doubleLogEntryClass.getMethod("append", double.class);

            for (int i = 0; i < keys.length; ++i) {
                Object entryObj = ctor.newInstance(dataLogInstance, keys[i]);
                temp[i] = new ReflectionEntry(entryObj, appendMethod);
            }
        } catch (Exception e) {
            // Reflection failed (datalog not available) -> fallback to SmartDashboard
            for (int i = 0; i < keys.length; ++i) {
                temp[i] = new SmartDashboardEntry(keys[i]);
            }
        }

        this.entries = temp;
    }

    // call this once per robotPeriodic (or subsystem periodic) to append values
    public void periodic() {
        // pilot
        safeAppend(0, pilot, Axis.LEFT_X);
        safeAppend(1, pilot, Axis.LEFT_Y);
        safeAppend(2, pilot, Axis.RIGHT_X);
        safeAppend(3, pilot, Axis.RIGHT_Y);

        // copilot
        safeAppend(4, copilot, Axis.LEFT_X);
        safeAppend(5, copilot, Axis.LEFT_Y);
        safeAppend(6, copilot, Axis.RIGHT_X);
        safeAppend(7, copilot, Axis.RIGHT_Y);

        // testController
        safeAppend(8, testController, Axis.LEFT_X);
        safeAppend(9, testController, Axis.LEFT_Y);
        safeAppend(10, testController, Axis.RIGHT_X);
        safeAppend(11, testController, Axis.RIGHT_Y);
    }

    private void safeAppend(int index, DualShock4Controller ctrl, Axis axis) {
        if (ctrl == null) {
            return;
        }
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
            entries[index].append(v);
        } catch (Exception ignored) {
            // protect logging from controller changes / runtime issues
        }
    }

    private enum Axis { LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y }

    // abstraction for a numeric log entry
    private interface LogEntry {
        void append(double v);
    }

    // reflection-backed entry for DoubleLogEntry
    private static class ReflectionEntry implements LogEntry {
        private final Object entryInstance;
        private final Method appendMethod;

        ReflectionEntry(Object entryInstance, Method appendMethod) {
            this.entryInstance = entryInstance;
            this.appendMethod = appendMethod;
        }

        @Override
        public void append(double v) {
            try {
                appendMethod.invoke(entryInstance, v);
            } catch (Exception ignored) {
            }
        }
    }

    // SmartDashboard-backed entry
    private static class SmartDashboardEntry implements LogEntry {
        private final String key;

        SmartDashboardEntry(String key) {
            this.key = key;
        }

        @Override
        public void append(double v) {
            SmartDashboard.putNumber(key, v);
        }
    }
}