package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.States;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter.Shooter;

public class DriverCommunications {
    static boolean CurrentHubState = Shooter.getInstance().isHubActive();

    public static void pushToElastic() {

        double PhaseTime = 0;
        Timer PhaseCountdown = new Timer();
        // If the hub state changes, reset the phase shift timer and change
        // currentHUbSTate
        if (!Shooter.getInstance().isHubActive() == CurrentHubState) {
            CurrentHubState = Shooter.getInstance().isHubActive();
            PhaseCountdown.restart();
        }

        // Detect when we're in a new based on game timer and set the appropriate time
        // limit
        if (Constants.timer.get() <= 10) {
            PhaseTime = 10;
        } else if ((Constants.timer.get() > 10) || (Constants.timer.get() <= 115)) {
            PhaseTime = 25;

        } else if ((Constants.timer.get() >= 116)) {
            PhaseTime = 30;

        }

        // set PhaseClock as the time before phase shift by subtracting timer from max
        // shift time
        double PhaseClock = (PhaseTime - PhaseCountdown.get());

        SmartDashboard.putNumber("Phase Shift Countdown", PhaseClock);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Is Hub Active?", Shooter.getInstance().isHubActive());
        SmartDashboard.putBoolean("In Manual?", Superstructure.getInstance().getCurrentState() == States.MANUAL);
        SmartDashboard.putBoolean("Can Shoot", Shooter.getInstance().Shootable());
    }
}
