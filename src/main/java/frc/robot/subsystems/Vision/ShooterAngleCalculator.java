package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterAngleCalculator {
    
    //contants
    private static final double g = 9.81; // gravity
    private static final double hubHeight = 1.52; // hub height
    private static final double ferryHeight = 0.5; // hub height
    private static final double minHoodAngle = 1.012; // 58 deg
    private static final double shooterHeight = 0.53; // shooter height
    private static final double hubRadius = 1.27; // radius of the hub
    private static final int maxNewtonsMethodIterations = 30; // prevents an ifinate loop 
    


    
    public static ShooterAngle getShooterAngleToFerry(double vx, double vy, double pfx, double pfy, double s){
        
        // needed height
        double pfz = ferryHeight - shooterHeight;

        // estimate t and theta for inital guess
        // first estimate theta
        double R = Math.sqrt(pfx * pfx + pfy * pfy);
        double theta = Math.atan((s * s + Math.sqrt(s*s*s*s - g*g*R*R - 2*pfz*g*s*s)) / (g * R));

        Logger.recordOutput("hood Ferry", theta);

        // // then estimate t
        double T = 0;//R / (s * Math.cos(theta));

        // double T = 0.3;

        // Newton's method
        double f = Integer.MAX_VALUE;
        int newtonsMethodIterations = 0;
        while (Math.abs(f) > 0.1){
            f = quarticFunction(T, vx, vy, pfx, pfy, pfz, s);
            T = T - f / quarticDerivative(T, vx, vy, pfx, pfy, pfz, s);
            newtonsMethodIterations = newtonsMethodIterations + 1;
            if (newtonsMethodIterations > maxNewtonsMethodIterations){
                return null;
            }  
        }

        double phxAim = pfx - vx * T;
        double phyAim = pfy - vy * T;

        R = Math.sqrt((phxAim*phxAim) + (phyAim*phyAim));
        theta = Math.atan(((s*s) + Math.sqrt((s*s*s*s) - (g*g) * (R*R) - 2 * pfz * g * (s*s))) / (g * R));
        
        //time when in front of the hub
        double th = (R-hubRadius)/(s * Math.cos(theta));
        //height when in front of the hub
        double ht = shooterHeight + th * s * Math.sin(theta) - 0.5 * T*T * g;

        if (theta > minHoodAngle){
                double phi;
            if (phyAim > 0){
                phi = Math.atan(phyAim / phxAim);
            } else {
                phi = Math.atan(phyAim / phxAim)+Math.PI;
            }
                
            return new ShooterAngle(phi, theta);
        }
        else
            return null;
        
    }
// paramaters are vx is the x velocity of the robot vy is the y velocity of the robot 
// phx is the x position of the hub in relation to the robot phy is the same for the y 
// s is the speed of the flywheels
    public static ShooterAngle getShooterAngleToHub(double vx, double vy, double phx, double phy, double s){
        // needed height
        double phz = hubHeight - shooterHeight;

        // SmartDashboard.putNumber("relativePositionToHubX",phx);
        // SmartDashboard.putNumber("relativePositionToHubY",phy);

        // estimate t and theta for inital guess
        // first estimate theta
        double R = Math.sqrt(phx * phx + phy * phy);
        double theta = Math.atan((s * s + Math.sqrt(s*s*s*s - g*g*R*R - 2*phz*g*s*s)) / (g * R));
        Logger.recordOutput("hood hub", theta);

        // then estimate t
        double T = R / (s * Math.cos(theta));

        // Newton's method
        double f = Integer.MAX_VALUE;
        int newtonsMethodIterations = 0;
        double[] newtons_method_results= new double[maxNewtonsMethodIterations];
        double[] newtons_method_derivative_results= new double[maxNewtonsMethodIterations];
        while (Math.abs(f) > 1e-2){
            f = quarticFunction(T, vx, vy, phx, phy, phz, s);
            double qd = quarticDerivative(T, vx, vy, phx, phy, phz, s);
            if (qd != 0)
                T = T - f / qd;
            newtonsMethodIterations = newtonsMethodIterations + 1;
            if (newtonsMethodIterations >= maxNewtonsMethodIterations){
                double phxAim = phx - vx * T;
                double phyAim = phy - vy * T;
                
                R = Math.sqrt((phxAim * phxAim) + (phyAim * phyAim));

                theta = Math.atan(((s*s) + Math.sqrt((s*s*s*s) - (g*g) * (R*R) - 2 * phz * g * (s*s))) / (g * R));
                Logger.recordOutput("hasSolution", false);
                return null;
            }
            newtons_method_results[newtonsMethodIterations] = f;
            newtons_method_derivative_results[newtonsMethodIterations] = qd;
        }

        //larger problums in our math than i thought (here we assume newtons method found the right angle)
        double phxAim = phx - vx * T;
        double phyAim = phy - vy * T;

        R = Math.sqrt((phxAim*phxAim) + (phyAim*phyAim));

        theta = Math.atan(((s*s) + Math.sqrt((s*s*s*s) - (g*g) * (R*R) - 2 * phz * g * (s*s))) / (g * R));

        // SmartDashboard.putString("ShooterAngleCalkDebug", "final val:" + f+" theta:"+theta);

        //time when in front of the hub
        double th = (R-hubRadius)/(s * Math.cos(theta));
        //height when in front of the hub
        double ht = shooterHeight + th * s * Math.sin(theta) - 0.5 * th*th * g;

        // SmartDashboard.putString("ShooterAngleCalkDebug", "ht" + ht +" theta:"+theta);

        if (ht > hubHeight){
                double phi;
            if (phxAim > 0){
                phi = Math.atan(phyAim / phxAim);
            } else {
                phi = Math.atan(phyAim / phxAim)+Math.PI;
            }
            //TODO needs be concerted to turret and hood relative values 
            Logger.recordOutput("hasSolution", true);  
            return new ShooterAngle(phi, theta);
            //phi is the angle of the turrent with respect to the field
            //theta is the angle of elevation of the hood
        }
        else
            Logger.recordOutput("hasSolution", false);
            return null;
    }

    private static double quarticFunction(double t, double vx, double vy, double phx, double phy, double phz, double s){
        return (0.25) * (g*g) * (t*t*t*t) + ((vx*vx) + (vy*vy) + g*phz - (s*s)) * (t*t) - 2 * (phx*vx + phy*vy) * t + (phx*phx) + (phy*phy) + (phz*phz);
    }

    private static double quarticDerivative(double t, double vx, double vy, double phx, double phy, double phz, double s){
        return (g*g) * (t*t*t) + 2 * t * ((vx*vx) + (vy*vy) + g * phz - (s*s)) - 2 * (phx * vx + phy * vy);
    }

}