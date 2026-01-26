package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.Shooter;

public class DriverCommunications{
    public static void pushToElastic(){
              SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
              SmartDashboard.putBoolean("Is Hub Active?", Shooter.getInstance().isHubActive());
    }
}
