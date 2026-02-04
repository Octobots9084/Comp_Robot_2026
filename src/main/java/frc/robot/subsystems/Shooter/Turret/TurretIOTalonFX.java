package frc.robot.subsystems.Shooter.Turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConfigurator;

public class TurretIOTalonFX implements TurretIO{
    public TalonFX hoodMotor;
    public TalonFX turretMotor;
    public ShooterConfigurator shooterConfigs;
    private MotionMagicVoltage hoodRequest;
    private MotionMagicVoltage turretRequest;

    public TurretIOTalonFX(){
        shooterConfigs = new ShooterConfigurator();
        hoodMotor = new TalonFX(Constants.hoodID, Constants.krakenBus);
        turretMotor = new TalonFX(Constants.turretID,Constants.krakenBus);

        hoodMotor.getConfigurator().apply(shooterConfigs.hoodConfig);
        turretMotor.getConfigurator().apply(shooterConfigs.turretConfig);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        inputs.hoodMotorTemp = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.turretMotorTemp = turretMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setTurretPosition(double turretAngle){
        turretRequest.Position = turretAngle;
        turretMotor.setControl(turretRequest);
    }

    @Override
    public void setHoodPosition(double hoodAngle){
        hoodRequest.Position = hoodAngle;
        hoodMotor.setControl(hoodRequest);
    }

    @Override
    public double getHoodPosition(){
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getTurretPosition(){
        return turretMotor.getPosition().getValueAsDouble();
    }
    
    @Override
    public boolean hoodInTolerance(double tolerance){
        return MathUtil.isNear(hoodRequest.getPositionMeasure().in(Units.Revolution), this.getHoodPosition(), tolerance);
    }

    @Override
    public boolean turretInTolerance(double tolerance){
        return MathUtil.isNear(turretRequest.getPositionMeasure().in(Units.Revolution), this.getTurretPosition(), tolerance);
    }

    /** loop this if it is being used */
    
    double startTime = -1000;
    double duration = 1000;

    public void startFiring() {
        startTime = System.currentTimeMillis();
    };

    // public void turretloop() {
    //     if (false /*turret is in incorrect state to fire*/) return;
    //     if (false /*fuel detected*/) startFiring();

    //     if (startTime + duration >= System.currentTimeMillis()) alignTurretToGoal();

    //     //turret is within 0.1 rotations of target
    //     if (!(getTurretPosition() + 0.05 > turretRequest.Position && getTurretPosition() - 0.05 < turretRequest.Position)) return;

        
    // }


}
