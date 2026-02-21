package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    public static boolean isBlueAlliance() {
        if (Constants.allianceColor != DriverStation.Alliance.Blue) {
            return false;
        } else {
            return true;
        }
    }
}
