//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

//Contains some utility functions and constants for Coordinate-based classes
public abstract class AbstractCoordinate extends AbstractCommand {
    public static final double LIMB_LENGTH = 2500;
    
    static double roboToRadians(double robo) {
        return Math.toRadians(robo / 100);
    }
    
    static double radiansToRobo(double radians) {
        return Math.toDegrees(radians) * 100;
    }
    
    //Takes any angle in robot units and converts it to its closest equivalent integer angle in the range (-18000,18000].
    static int normalize (double angle) {
        long roboAngle = Math.round(angle);
        roboAngle %= 36000;
        if (roboAngle <= -18000) roboAngle += 36000;
        if (roboAngle > 18000) roboAngle -= 36000;
        return (int)roboAngle;
    }
}
