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
        
        SmartDashboard.putString("Field/Current Shift", currentShift);

        // 4. Scoring Eligibility 
        // Example: logic to see if our alliance can currently score based on the shift
        boolean canScore = isOurShift(alliance, currentShift);
        SmartDashboard.putBoolean("Field/Can Score", canScore);
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