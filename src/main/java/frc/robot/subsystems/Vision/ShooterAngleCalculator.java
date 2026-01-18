package frc.robot.subsystems.Vision;

public class ShooterAngleCalculator {
    //contants
    private final double g = 9.81; // gravity
    private final double hubHeight = 1.8288; // hub height
    private final double shooterHeight = 0.4572; // shooter height
    private final int maxNewtonsMethodIterations = 30; // prevents an ifinate loop 
    

    public ShooterAngle getShooterAngle(double vx, double vy, double phx, double phy, double s){
        // needed height
        double phz = hubHeight - shooterHeight;

        // estimate t and theta for inital guess
        // first estimate theta
        double R = Math.sqrt(phx * phx + phy * phx);
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


        double phxAim = phx - vx * T;
        double phyAim = phy - vy * T;

        R = Math.sqrt((phxAim*phxAim) + (phyAim*phyAim));
        theta = Math.atan(((s*s) + Math.sqrt((s*s*s*s) - (g*g) * (R*R) - 2 * phz * g * (s*s))) / (g * R));
        double phi;
        if (phyAim > 0){
            phi = Math.atan(phyAim / phxAim);
        } else {
            phi = Math.atan(phyAim / phxAim)+Math.PI;
        }
            
        return new ShooterAngle(phi, theta);
    }

    private double quarticFunction(double t, double vx, double vy, double phx, double phy, double phz, double s){
        return (1/4) * (g*g) * (t*t*t*t) + ((vx*vx) + (vy*vy) + g*phz - (s*s)) * (t*t) - 2 * (phx*vx + phy*vy) * t + (phx*phx) + (phy*phy) + (phz*phz);
    }

    private double quarticDerivative(double t, double vx, double vy, double phx, double phy, double phz, double s){
        return (g*g) * (t*t*t) + 2 * ((vx*vx) + (vy*vy) + g * phz - (s*s)) - 2 * (phx * vx + phy * vy);
    }

}