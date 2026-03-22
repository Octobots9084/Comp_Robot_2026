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

    static {
        //distance, hoodangle
        hoodAngleMapHub.put(3.033494684,new Rotation2d(72.0*Math.PI/180.0));
        hoodAngleMapHub.put(3.45835351,new Rotation2d(67.0*Math.PI/180.0));
        hoodAngleMapHub.put(4.057697993,new Rotation2d(68.0*Math.PI/180.0));
        hoodAngleMapHub.put(4.574690044,new Rotation2d(72.0*Math.PI/180.0));
        hoodAngleMapHub.put(5.540799311,new Rotation2d(66.5*Math.PI/180.0));
        //distance, flywheel speed
        flywheelSpeedMapHub.put(3.033494684,9.3);
        flywheelSpeedMapHub.put(3.45835351,9.3);
        flywheelSpeedMapHub.put(4.057697993,9.9);
        flywheelSpeedMapHub.put(4.574690044,10.6);
        flywheelSpeedMapHub.put(5.540799311,11.3);
        //distance, time
        timeOfFlightMapHub.put(3.033494684,1.038);
        timeOfFlightMapHub.put(3.45835351,1.033);
        timeOfFlightMapHub.put(4.057697993,1.054);
        timeOfFlightMapHub.put(4.574690044,1.22);
        timeOfFlightMapHub.put(5.540799311,1.32);

        //distance, hoodangle
        hoodAngleMapFerry.put(3.033494684,new Rotation2d(72.0*Math.PI/180.0));
        hoodAngleMapFerry.put(3.45835351,new Rotation2d(67.0*Math.PI/180.0));
        hoodAngleMapFerry.put(4.057697993,new Rotation2d(68.0*Math.PI/180.0));
        hoodAngleMapFerry.put(4.574690044,new Rotation2d(72.0*Math.PI/180.0));
        hoodAngleMapFerry.put(5.540799311,new Rotation2d(66.5*Math.PI/180.0));
        //distance, flywheel speed
        flywheelSpeedMapFerry.put(3.033494684,9.3);
        flywheelSpeedMapFerry.put(3.45835351,9.3);
        flywheelSpeedMapFerry.put(4.057697993,9.9);
        flywheelSpeedMapFerry.put(4.574690044,10.6);
        flywheelSpeedMapFerry.put(5.540799311,11.3);
        //distance, time
        timeOfFlightMapFerry.put(3.033494684,1.038);
        timeOfFlightMapFerry.put(3.45835351,1.033);
        timeOfFlightMapFerry.put(4.057697993,1.054);
        timeOfFlightMapFerry.put(4.574690044,1.22);
        timeOfFlightMapFerry.put(5.540799311,1.32);
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
        }

        double RealX = XToHub + vx * T;
        double RealY = YToHub + vy * T;
        double RealD = Pythgorian(RealX, RealY);

        double angleToHub;
        if (XToHub> 0 )
            angleToHub = Math.atan(RealY/RealX) % Math.PI*2;
        else
            angleToHub = (Math.atan(RealY/RealX)+ Math.PI) % Math.PI*2;

        return new ShooterAngle(angleToHub , hoodAngleMap.get(RealD).getRadians(), flywheelSpeedMap.get(RealD));
    }

    public static ShooterAngle getShooterAngle(double XToHub, double YToHub){
        double d = Pythgorian(XToHub, YToHub);
        if (XToHub == 0)
            return new ShooterAngle(0 , 0, 0);
        double angleToHub;
        if (XToHub> 0 )
            angleToHub = Math.atan(YToHub/XToHub) % Math.PI*2;
        else
            angleToHub = (Math.atan(YToHub/XToHub)+ Math.PI) % Math.PI*2;
        
        return new ShooterAngle(angleToHub , hoodAngleMapHub.get(d).getRadians(), flywheelSpeedMapHub.get(d));
    }


    public static double getTOFError(double t,double x,double y,double vx,double vy, InterpolatingDoubleTreeMap timeOfFlightMap){
        double predDist = Pythgorian(x+vx*t,y+vy*t);
        double predTime = timeOfFlightMap.get(predDist);
        double error = t - predTime;
        return error;
    }

    public static double getTOFErrorFerry(double t,double x,double y,double vx,double vy, InterpolatingDoubleTreeMap timeOfFlightMap){
        double predDist = Pythgorian(x+vx*t,y+vy*t);
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