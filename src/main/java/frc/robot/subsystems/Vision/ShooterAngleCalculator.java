package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;

public class ShooterAngleCalculator {
    
    //contants
    private static final int maxNewtonsMethodIterations = 30; // prevents an ifinate loop 
    public static final double lagTime = 0.1;
    
    // Hub LOTs
    public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapHub =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap flywheelSpeedMapHub =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMapHub =
        new InterpolatingDoubleTreeMap();

    // Ferrying LOTs
    public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapFerry =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap flywheelSpeedMapFerry =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMapFerry =
        new InterpolatingDoubleTreeMap();


    /*
    1.442809759
5.506476197
2.01922287
2.511019912
3.034157419
3.518755036
4.076567183
4.515152268
5.098166337
    */ 
    static {
        //distance, hoodangle
        hoodAngleMapHub.put(1.442809759,new Rotation2d(75.0*Math.PI/180.0));
        hoodAngleMapHub.put(2.01922287,new Rotation2d(73.0*Math.PI/180.0));
        hoodAngleMapHub.put(2.511019912,new Rotation2d(71.0*Math.PI/180.0));
        hoodAngleMapHub.put(3.034157419,new Rotation2d(69*Math.PI/180.0));
        hoodAngleMapHub.put(3.518755036,new Rotation2d(67.0*Math.PI/180.0));
        hoodAngleMapHub.put(4.076567183,new Rotation2d(65.0*Math.PI/180.0));
        hoodAngleMapHub.put(4.515152268,new Rotation2d(63.0*Math.PI/180.0));
        hoodAngleMapHub.put(5.098166337,new Rotation2d(61.0*Math.PI/180.0));
        hoodAngleMapHub.put(5.506476197,new Rotation2d(58.0*Math.PI/180.0));

        //distance, flywheel speed
        flywheelSpeedMapHub.put(1.442809759,26.0);
        flywheelSpeedMapHub.put(2.01922287,27.5);
        flywheelSpeedMapHub.put(2.511019912,29.0);
        flywheelSpeedMapHub.put(3.034157419,30.0);
        flywheelSpeedMapHub.put(3.518755036,31.0);
        flywheelSpeedMapHub.put(4.076567183,32.5);
        flywheelSpeedMapHub.put(4.515152268,33.5);
        flywheelSpeedMapHub.put(5.098166337,35.0);
        flywheelSpeedMapHub.put(5.506476197,35.5);

        //distance, time
        timeOfFlightMapHub.put(1.442809759,1.103235);
        timeOfFlightMapHub.put(2.511019912,1.185075);
        timeOfFlightMapHub.put(2.550176464,1.225);
        timeOfFlightMapHub.put(3.034157419,1.2666825);
        timeOfFlightMapHub.put(3.518755036,1.28165);
        timeOfFlightMapHub.put(4.076567183,1.3475);
        timeOfFlightMapHub.put(4.515152268,1.3514925);
        timeOfFlightMapHub.put(5.506476197,1.374175);

        //distance, hoodangle TODO MUST GATER DATA!
        hoodAngleMapFerry.put(3.05239922,new Rotation2d(58.0*Math.PI/180.0));
        hoodAngleMapFerry.put(9.291783575,new Rotation2d(58.0*Math.PI/180.0));
        //distance, flywheel speed TODO MUST GATER DATA!
        flywheelSpeedMapFerry.put(3.05239922,22.5);
        flywheelSpeedMapFerry.put(9.291783575,39.5);
        //distance, time TODO MUST GATER DATA!
        timeOfFlightMapFerry.put(3.05239922,1.085);//no TOF data yet
        timeOfFlightMapFerry.put(9.291783575,1.63);
    }
    
    // d, vx,vy are robot to hub reletive
    public static ShooterAngle getShooterAngle(double vx, double vy, double XToHub, double YToHub, InterpolatingDoubleTreeMap flywheelSpeedMap, InterpolatingDoubleTreeMap timeOfFlightMap, InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap){
        double d = Pythgorian(XToHub, YToHub);
        double T = timeOfFlightMap.get(d);
        double TOFError = 90000;
        int count = 0;
        while(TOFError > 0.001){
            
            T = T - (getTOFError(T, XToHub, YToHub, vx, vy, timeOfFlightMap) / getTOFErrorDerivative(T, XToHub, YToHub, vx, vy));

            TOFError = getTOFError(T, XToHub, YToHub, vx, vy, timeOfFlightMap);

            if (count >= maxNewtonsMethodIterations)
                break;
            else
                count++;
            Logger.recordOutput("Newtons method iterations",count);
        }

        double RealX = XToHub - vx * T;
        double RealY = YToHub - vy * T;
        double RealD = Pythgorian(RealX, RealY);

        double angleToHub;
        if (XToHub> 0 )
            angleToHub = Math.atan(RealY/RealX);
        else
            angleToHub = (Math.atan(RealY/RealX)+ Math.PI);

        return new ShooterAngle(angleToHub , hoodAngleMap.get(RealD).getRadians(), flywheelSpeedMap.get(RealD));
    }

    public static ShooterAngle getShooterAngle(double XToHub, double YToHub){
        double d = Pythgorian(XToHub, YToHub);
        if (XToHub == 0)
            return new ShooterAngle(0 , 0, 0);
        double angleToHub;
        if (XToHub> 0 )
            angleToHub = Math.atan(YToHub/XToHub);
        else
            angleToHub = (Math.atan(YToHub/XToHub)+ Math.PI);
        
        return new ShooterAngle(angleToHub , hoodAngleMapHub.get(d).getRadians(), flywheelSpeedMapHub.get(d));
    }


    public static double getTOFError(double t,double x,double y,double vx,double vy, InterpolatingDoubleTreeMap timeOfFlightMap){
        double predDist = Pythgorian(x-vx*t,y-vy*t);
        double predTime = timeOfFlightMap.get(predDist);
        double error = t - predTime;
        return error;
    }


    public static double getTOFErrorDerivative(double t,double x,double y,double vx,double vy){
        double d = Pythgorian(x, y);
        double errorDerivative = 1 + (x*vx+y*vy)/((d*d/t));

        return errorDerivative;
    }

    public static double Pythgorian(double a,double b){
        return Math.sqrt(a*a+b*b);
    }
        
}