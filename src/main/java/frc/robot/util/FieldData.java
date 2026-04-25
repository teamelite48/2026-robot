package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * Utility class to manage and broadcast match-specific data 
 * for the "Rebuilt" game.
 */
public class FieldData {

    /**
     * Updates and sends all relevant field data to the SmartDashboard.
     * Call this inside robotPeriodic() to ensure real-time updates.
     */
    public static void update() {

        double matchTime = DriverStation.getMatchTime();
        String currentShiftName = "None";
        double shiftTimeRemaining = 0;

        // 1. Match Time
        double time = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Field/Match Time", time);

        // 2. Alliance Info
        Optional<Alliance> alliance = DriverStation.getAlliance();
        String allianceColor = alliance.isPresent() ? alliance.get().toString() : "Disconnected";
        int station = DriverStation.getLocation().isPresent() ? DriverStation.getLocation().getAsInt() : 0;
        
        SmartDashboard.putString("Field/Alliance", allianceColor);
        SmartDashboard.putNumber("Field/Station Number", station);

        // 3. Game Phase (Shifts)
        String gameMessage = DriverStation.getGameSpecificMessage();
        String currentShift = "None";

        if (gameMessage != null && !gameMessage.isEmpty()) {
            currentShift = gameMessage.toUpperCase();
        }

        // 4. Scoring Eligibility 
        // Example: logic to see if our alliance can currently score based on the shift
        boolean canScore = isOurShift(alliance, currentShift);
        SmartDashboard.putBoolean("Field/Can Score", canScore);

        if (DriverStation.isAutonomous()) {
            currentShiftName = "AUTONOMOUS";
            shiftTimeRemaining = matchTime; // 20 -> 0
        } 
        else if (DriverStation.isTeleop()) {
            // Teleop logic ladder based on your 140s total window
            if (matchTime > 130) {
                currentShiftName = "TRANSITION";
                shiftTimeRemaining = matchTime - 130; // 140 -> 130 (10s)
            }
            else if (matchTime > 105) {
                currentShiftName = "SHIFT 1";
                shiftTimeRemaining = matchTime - 105; // 130 -> 105 (25s)
            }
            else if (matchTime > 80) {
                currentShiftName = "SHIFT 2";
                shiftTimeRemaining = matchTime - 80;  // 105 -> 80 (25s)
            }
            else if (matchTime > 55) {
                currentShiftName = "SHIFT 3";
                shiftTimeRemaining = matchTime - 55;  // 80 -> 55 (25s)
            }
            else if (matchTime > 30) {
                currentShiftName = "SHIFT 4";
                shiftTimeRemaining = matchTime - 30;  // 55 -> 30 (25s)
            }
            else {
                currentShiftName = "ENDGAME";
                shiftTimeRemaining = matchTime;       // 30 -> 0 (30s)
            }
        }

        SmartDashboard.putNumber("Field/Shift Timer", Math.max(0, shiftTimeRemaining));
        SmartDashboard.putString("Field/Current Shift", currentShiftName);

    }

    /**
     * Helper to determine if the current game shift matches our alliance.
     */
    private static boolean isOurShift(Optional<Alliance> alliance, String shift) {
        if (alliance.isEmpty()) return false;
        
        if (alliance.get() == Alliance.Red && shift.contains("RED")) return true;
        if (alliance.get() == Alliance.Blue && shift.contains("BLUE")) return true;
        
        return false;
    }
}