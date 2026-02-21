package frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Shooter.Turret.TurretIO.TurretIOInputs;

public class ShooterIOSystem implements ShooterIO {
    private Shooter shooterMain;

    public ShooterIOSystem(Shooter shooterMain) {
        this.shooterMain = shooterMain;
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.ShooterCurrentState = Shooter.getInstance().currentShooterState;
    }

}
