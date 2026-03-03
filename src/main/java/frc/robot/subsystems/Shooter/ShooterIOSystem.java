package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Turret.TurretIO.TurretIOInputs;

public class ShooterIOSystem implements ShooterIO {
    private Shooter shooterMain;

    public ShooterIOSystem(Shooter shooterMain) {
        this.shooterMain = shooterMain;
    }

    public void updateInputs(ShooterIOInputs inputs) {
        Shooter shooter = Shooter.getInstance();
        inputs.ShooterCurrentState = shooter.currentShooterState;
        inputs.ShooterWantedState = shooter.wantedShooterState;
        inputs.turretHubAngle = shooter.turretHubAngle;
        inputs.hoodHubAngle = shooter.hoodHubAngle;
        inputs.isAimedAtHub = shooter.isAimedAtHub;
        inputs.flywheelCalculatorVelocity = shooter.shooterCalculatorVelocity;
        inputs.timer = Constants.timer.get();
    }

}
