
public class ConfigurationCoordinate extends AbstractCoordinate {
    //These variables are expressed in robot units, where:
    //1 cm = 100 robot units, and
    //1 degree = 100 robot units
    public final double waist, shoulder, elbow, wrist, hand;
    
    //The camera is not anchored exactly on the robot, so this offset is required to level it.
    static int HAND_OFFSET = 6525;
    
    public ConfigurationCoordinate(double waist, double shoulder, double elbow, double wrist, double hand) {
        this.waist = waist;
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.wrist = wrist;
        this.hand = hand;
    }
    
    //Copy constructor
    public ConfigurationCoordinate(ConfigurationCoordinate o) {
        this(o.waist, o.shoulder, o.elbow, o.wrist, o.hand);
    }
    
    //Inverse kinematics constructor
    public ConfigurationCoordinate(WorkspaceCoordinate o) {
        double waist = Math.atan2(o.x, o.y);
        double horDist = Math.sqrt(o.x * o.x + o.y * o.y);
        double dist = Math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
        double z = o.z;
        double centerAngle = Math.acos((2 * LIMB_LENGTH * LIMB_LENGTH - dist * dist) / (2 * LIMB_LENGTH * LIMB_LENGTH));
        double elbow = Math.PI - centerAngle;
        double shouldToWrist = Math.atan2(horDist, z);
        double otherAngle = Math.asin((LIMB_LENGTH / dist) * Math.sin(centerAngle));
        double shoulder = shouldToWrist - otherAngle;
        double wristPointAngle = shoulder + elbow;
        double desiredWrist = (Math.PI / 2) - roboToRadians(o.wrist);
        double wrist = desiredWrist - wristPointAngle;
        
        //Unfortunately, I can't call a constructor down here.
        this.waist = radiansToRobo(waist);
        this.shoulder = radiansToRobo(shoulder);
        this.elbow = radiansToRobo(elbow);
        this.wrist = radiansToRobo(wrist);
        this.hand = o.handTwist;
    }
    
    @Override
    public String getUnescapedCommandString() {
        return String.format("%d %d %d %d %d AJMA", normalize(hand + HAND_OFFSET), normalize(wrist), normalize(elbow), normalize(shoulder), normalize(waist));
    }
}
