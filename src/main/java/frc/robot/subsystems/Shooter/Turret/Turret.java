package frc.robot.subsystems.Shooter.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    public static Turret currentInstance = null;

    public final double spitTurrentAnge = 0.25;
    
    public final double spitTurrentHood = 0.67;//hehe change this to real or it'll be 67

    public final double spitTurrentOutAnge = 0.55;//todo

    public TurretIO io = new TurretIOTalonFX();

    public Turret(){
        currentInstance = this;
    }

    public static Turret getInstance(){
        return currentInstance;
    }

    public static void setInstance(Turret instance){
        currentInstance = instance;
    }

    public void setTurretPosition(double turretAngle){
        io.setTurretPosition(turretAngle);
    }

    public void setHoodPosition(double hoodAngle){
        io.setHoodPosition(hoodAngle);
    }

    public double getHoodPosition(){
        return io.getHoodPosition();
    }
    public double getTurretPosition(){
        return io.getTurretPosition();
    }

    public boolean hoodInTolerance(double tolerance){
        return io.hoodInTolerance(tolerance);
    }

    public boolean turretInTolerance(double tolerance){
        return io.turretInTolerance(tolerance);
    }
}
