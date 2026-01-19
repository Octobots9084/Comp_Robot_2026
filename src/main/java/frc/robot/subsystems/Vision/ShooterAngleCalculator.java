package frc.robot.subsystems.Vision;

public class ShooterAngleCalculator {
    
    //contants
    private final double g = 9.81; // gravity
    private final double hubHeight = 1.8288; // hub height
    private final double ferryHeight = 0.5; // hub height
    private final double minHoodAngle = 1.012; // 58 deg
    private final double shooterHeight = 0.4572; // shooter height
    private final double hubRadius = 1.27; // radius of the hub
    private final int maxNewtonsMethodIterations = 30; // prevents an ifinate loop 
    
    public ShooterAngle getShooterAngleToFerry(double vx, double vy, double pfx, double pfy, double s){
        // needed height
        double pfz = ferryHeight - shooterHeight;

        // estimate t and theta for inital guess
        // first estimate theta
        double R = Math.sqrt(pfx * pfx + pfy * pfy);
        double theta = Math.atan((s * s + Math.sqrt(s*s*s*s - g*g*R*R - 2*pfz*g*s*s)) / (g * R));

        // then estimate t
        double T = R / (s * Math.cos(theta));

        // Newton's method
        double f = Integer.MAX_VALUE;
        int newtonsMethodIterations = 0;
        while (Math.abs(f) > 1e-6){
            f = quarticFunction(T, vx, vy, pfx, pfy, pfz, s);
            T = T - f / quarticDerivative(T, vx, vy, pfx, pfy, pfz, s);
            newtonsMethodIterations = newtonsMethodIterations + 1;
            if (newtonsMethodIterations < maxNewtonsMethodIterations){
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

        else return null;
        
    }

    public ShooterAngle getShooterAngleToHub(double vx, double vy, double phx, double phy, double s){
        // needed height
        double phz = hubHeight - shooterHeight;

        // estimate t and theta for inital guess
        // first estimate theta
        double R = Math.sqrt(phx * phx + phy * phy);
        double theta = Math.atan((s * s + Math.sqrt(s*s*s*s - g*g*R*R - 2*phz*g*s*s)) / (g * R));

        // then estimate t
        double T = R / (s * Math.cos(theta));

        // Newton's method
        double f = Integer.MAX_VALUE;
        int newtonsMethodIterations = 0;
        while (Math.abs(f) > 1e-6){
            f = quarticFunction(T, vx, vy, phx, phy, phz, s);
            T = T - f / quarticDerivative(T, vx, vy, phx, phy, phz, s);
            newtonsMethodIterations = newtonsMethodIterations + 1;
            if (newtonsMethodIterations < maxNewtonsMethodIterations){
                return null;
            }  
        }

        //larger problums in our math than i thought (here we assume newtons method found the right angle)
        double phxAim = phx - vx * T;
        double phyAim = phy - vy * T;

        R = Math.sqrt((phxAim*phxAim) + (phyAim*phyAim));
        theta = Math.atan(((s*s) + Math.sqrt((s*s*s*s) - (g*g) * (R*R) - 2 * phz * g * (s*s))) / (g * R));
        
        //time when in front of the hub
        double th = (R-hubRadius)/(s * Math.cos(theta));
        //height when in front of the hub
        double ht = shooterHeight + th * s * Math.sin(theta) - 0.5 * T*T * g;

        if (ht > hubHeight){
                double phi;
            if (phyAim > 0){
                phi = Math.atan(phyAim / phxAim);
            } else {
                phi = Math.atan(phyAim / phxAim)+Math.PI;
            }
                
            return new ShooterAngle(phi, theta);
        }

        else return null;
        
    }

    private double quarticFunction(double t, double vx, double vy, double phx, double phy, double phz, double s){
        return (1/4) * (g*g) * (t*t*t*t) + ((vx*vx) + (vy*vy) + g*phz - (s*s)) * (t*t) - 2 * (phx*vx + phy*vy) * t + (phx*phx) + (phy*phy) + (phz*phz);
    }

    private double quarticDerivative(double t, double vx, double vy, double phx, double phy, double phz, double s){
        return (g*g) * (t*t*t) + 2 * t * ((vx*vx) + (vy*vy) + g * phz - (s*s)) - 2 * (phx * vx + phy * vy);
    }

}