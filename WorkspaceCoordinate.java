//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

public class WorkspaceCoordinate extends AbstractCoordinate {
    //These variables are expressed in robot units, where:
    //1 cm = 100 robot units, and
    //1 degree = 100 robot units
    public final double x, y, z, wrist, handTwist;
    
    //The camera is not anchored exactly on the robot, so this offset is required to level it.
    //This offset is different than the offset in ConfigurationCoordinate because Dr. Silaghi added an offset of his own to the TMOVETO command.
    static int HAND_OFFSET = 3000;
    
    public WorkspaceCoordinate(double x, double y, double z, double wrist, double handTwist) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.wrist = wrist;
        this.handTwist = handTwist;
    }
    
    //Copy constructor
    public WorkspaceCoordinate(WorkspaceCoordinate o) {
        this(o.x, o.y, o.z, o.wrist, o.handTwist);
    }
    
    //Kinematics constructor
    public WorkspaceCoordinate(ConfigurationCoordinate o) {
        double horDist = 0;
        double z = 0;
        double angle = roboToRadians(o.shoulder);
        horDist += Math.sin(angle) * LIMB_LENGTH;
        z += Math.cos(angle) * LIMB_LENGTH;
        angle += roboToRadians(o.elbow);
        horDist += Math.sin(angle) * LIMB_LENGTH;
        z += Math.cos(angle) * LIMB_LENGTH;
        double twist = roboToRadians(o.waist);
        double x = Math.sin(twist) * horDist;
        double y = Math.cos(twist) * horDist;
        angle += roboToRadians(o.wrist);
        
        //Unfortunately, I can't call a constructor down here.
        this.x = x;
        this.y = y;
        this.z = z;
        this.wrist = -(radiansToRobo(angle) - 9000);
        this.handTwist = o.hand;
    }

    @Override
    public String getUnescapedCommandString() {
        return String.format("%d %d %d %d %d TMOVETO", normalize(handTwist + HAND_OFFSET), normalize(wrist), Math.round(x), Math.round(y), Math.round(z));
    }
}
