//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.channels.Channels;
import java.nio.channels.FileChannel;
import java.nio.channels.ReadableByteChannel;
import java.util.List;
import java.util.ListIterator;

public class Main {
    //These variables are expressed in robot units, where:
    //1 cm = 100 robot units, and
    //1 degree = 100 robot units
    static final int X_LOW = -4500;
    static final int X_HIGH = -3800;
    static final int Y_LOW = -550;
    static final int Y_HIGH = 550;
    static final int Z_LOW = -1800; //This is specified as -1000 in the assignment, but empirically it is -2000. (Although the robot arm can't reach that low.)
    static final int Z_HIGH = -1000; //This is specified as  in the assignment, but empirically it is -1000.

    static final int HAND_LENGTH = 1000;
    static final int CAPTURE_DISTANCE = 300;
    static final int CAPTURE_OFFSET = HAND_LENGTH + CAPTURE_DISTANCE;
    
    static final int CAPTURE_DIAMETER = 100;
    static final int CAPTURE_RADIUS = CAPTURE_DIAMETER / 2;
    
    
    static final boolean PRINT_DEBUG_STATEMENTS = true;
    static final Logger log = new Logger(System.out, PRINT_DEBUG_STATEMENTS);
    
    static String password;
    
    public static void main (String[] args) throws IOException {
        password = args[0];
        CommandList commandList = buildCommands();
        commandList.addAll(inverted(commandList));
        sendCommands(commandList);
        downloadImages(commandList);
        deleteImages();
    }
    
    static boolean testKinematics() {
        CommandList commandList = buildCommands();
        commandList.addAll(inverted(commandList));
        return testKinematics(commandList);
    }
    
    //Tests the kinematic systems against eachother. So, if this test passes, at least they're consistent with eachother.
    static boolean testKinematics(CommandList commandList) {
        for (Command command : commandList) {
            if (command instanceof WorkspaceCoordinate) {
                WorkspaceCoordinate wc = (WorkspaceCoordinate)command;
                WorkspaceCoordinate recovered = new WorkspaceCoordinate(new ConfigurationCoordinate(wc));
                if (!recovered.getUnescapedCommandString().equals(wc.getUnescapedCommandString())) {
                    log.printf("%s != %s%n", recovered.getUnescapedCommandString(), wc.getUnescapedCommandString());
                    return false;
                }
            } else if (command instanceof ConfigurationCoordinate) {
                ConfigurationCoordinate cc = (ConfigurationCoordinate)command;
                ConfigurationCoordinate recovered = new ConfigurationCoordinate(new WorkspaceCoordinate(cc));
                if (!recovered.getUnescapedCommandString().equals(cc.getUnescapedCommandString())) {
                    log.printf("%s != %s%n", recovered.getUnescapedCommandString(), cc.getUnescapedCommandString());
                    return false;
                }
            }
        }
        return true;
    }
    
    static CommandList buildCommands() {
        //The prep command gets the robot into position so it doesn't hit the box on the way down.
        Command prepCommand = new WorkspaceCoordinate(0, -X_HIGH - CAPTURE_OFFSET, Z_LOW + CAPTURE_RADIUS, 0, 0);
        CommandList a = captureFront();
        Command blendCaptureCommand = new CaptureCommand();
        CommandList b = captureTop();
        //CommandList c = captureBack();
        CommandList commandList = new CommandList();
        commandList.add(new HomeCommand());
        commandList.add(prepCommand);
        commandList.addAll(a);
        commandList.add(zxBlend(a, b));
        commandList.add(blendCaptureCommand); //Not strictly needed, but I like the continuity.
        commandList.addAll(b);
        //commandList.add(xzBlend(b, c));
        //commandList.add(new CaptureCommand()); //Not strictly needed, but I like the continuity.
        //commandList.addAll(c);
        commandList.add(new HomeCommand());
        return commandList;
    }
    
    static CommandList inverted(CommandList commandList) {
        CommandList inverted = new CommandList();
        for (Command command : commandList) {
            if (command instanceof WorkspaceCoordinate) {
                inverted.add(new ConfigurationCoordinate((WorkspaceCoordinate)command)); //inverse kinematics
            } else if (command instanceof ConfigurationCoordinate) {
                inverted.add(new WorkspaceCoordinate((ConfigurationCoordinate)command)); //kinematics
            } else if (command instanceof CaptureCommand) {
                inverted.add(new CaptureCommand()); //Not repeating image IDs
            } else {
                inverted.add(command);
            }
        }
        return inverted;
    }
    
    //Transitions z and then x.
    static WorkspaceCoordinate zxBlend(WorkspaceCoordinate a, WorkspaceCoordinate b) {
        return new WorkspaceCoordinate(a.x, (a.y + b.y) / 2, b.z, (a.wrist + b.wrist) / 2, (a.handTwist + b.handTwist) / 2);
    }
    
    static WorkspaceCoordinate zxBlend(CommandList a, CommandList b) {
        return zxBlend(getLast(a, WorkspaceCoordinate.class), getFirst(b, WorkspaceCoordinate.class));
    }

    //Transitions x and then z.
    static WorkspaceCoordinate xzBlend(WorkspaceCoordinate a, WorkspaceCoordinate b) {
        return new WorkspaceCoordinate(b.x, (a.y + b.y) / 2, a.z, (a.wrist + b.wrist) / 2, (a.handTwist + b.handTwist) / 2);
    }
    
    static WorkspaceCoordinate xzBlend(CommandList a, CommandList b) {
        return xzBlend(getLast(a, WorkspaceCoordinate.class), getFirst(b, WorkspaceCoordinate.class));
    }
    
    @SuppressWarnings("unchecked")
    static <T> T getFirst(List<?> objects, Class<T> clazz) {
        for (Object object : objects) if (clazz.isInstance(object)) return (T) object;
        return null;
    }
    
    @SuppressWarnings("unchecked")
    static <T> T getLast(List<?> objects, Class<T> clazz) {
        ListIterator<?> iterator = objects.listIterator(objects.size());
        while (iterator.hasPrevious()) {
            Object object = iterator.previous();
            if (clazz.isInstance(object)) return (T) object;
        }
        return null;
    }
    
    static void captureFront(CommandList commandList) {
        int yStart = Y_LOW + CAPTURE_RADIUS;
        int yEnd = Y_HIGH - CAPTURE_RADIUS;
        int yStep = CAPTURE_DIAMETER;
        boolean goingForward = true;
        for (int z = Z_LOW + CAPTURE_RADIUS; z <= Z_HIGH - CAPTURE_RADIUS; z += CAPTURE_DIAMETER) {
            for (int y = goingForward ? yStart : yEnd; goingForward ? y <= yEnd : y >= yStart; y += goingForward ? yStep : -yStep) {
                double length = length(X_HIGH, y);
                double shortenedLength = length - CAPTURE_OFFSET;
                double ratio = shortenedLength / length;
                int xTarget = (int)Math.round(X_HIGH * ratio);
                int yTarget = (int)Math.round(y * ratio);
                commandList.add(new WorkspaceCoordinate(xTarget, yTarget, z, 0, 0));
                commandList.add(new CaptureCommand());
            }
            goingForward = !goingForward;
        }
    }
    
    static CommandList captureFront() {
        CommandList commandList = new CommandList();
        captureFront(commandList);
        return commandList;
    }
    
    static double length(double a, double b) {
        return Math.sqrt(a * a + b * b);
    }
    
    static void captureTop(CommandList commandList) {
        int yStart = Y_LOW + CAPTURE_RADIUS;
        int yEnd = Y_HIGH - CAPTURE_RADIUS;
        int yStep = CAPTURE_DIAMETER;
        boolean goingForward = true;
        for (int x = X_HIGH - CAPTURE_RADIUS; x >= X_LOW + CAPTURE_RADIUS; x -= CAPTURE_DIAMETER) {
            for (int y = goingForward ? yStart : yEnd; goingForward ? y <= yEnd : y >= yStart; y += goingForward ? yStep : -yStep) {
                double angleRadians = -Math.atan2(y, -x);
                double angleDegrees = angleRadians * 180 / Math.PI;
                //log.printf("x: %d%ny: %d%nangle: %.3f%n%n", x, y, angleDegrees);
                int angleRobotUnits = (int)Math.round(angleDegrees * 100);
                commandList.add(new WorkspaceCoordinate(x, y, Z_HIGH + CAPTURE_OFFSET, -9000, angleRobotUnits));
                commandList.add(new CaptureCommand());
            }
            goingForward = !goingForward;
        }
    }
    
    static CommandList captureTop() {
        CommandList commandList = new CommandList();
        captureTop(commandList);
        return commandList;
    }
    
    static void captureBack(CommandList commandList) {
        int yStart = Y_LOW + CAPTURE_RADIUS;
        int yEnd = Y_HIGH - CAPTURE_RADIUS;
        int yStep = CAPTURE_DIAMETER;
        boolean goingForward = false;
        for (int z = Z_HIGH - CAPTURE_RADIUS; z >= Z_LOW + CAPTURE_RADIUS; z -= CAPTURE_DIAMETER) {
            for (int y = goingForward ? yStart : yEnd; goingForward ? y <= yEnd : y >= yStart; y += goingForward ? yStep : -yStep) {
                double length = length(X_LOW, y);
                double lengthenedLength = length + CAPTURE_OFFSET;
                double ratio = lengthenedLength / length;
                int xTarget = (int)Math.round(X_LOW * ratio);
                int yTarget = (int)Math.round(y * ratio);
                commandList.add(new WorkspaceCoordinate(xTarget, yTarget, z, -18000, 0));
                commandList.add(new CaptureCommand());
            }
            goingForward = !goingForward;
        }
    }
    
    static CommandList captureBack() {
        CommandList commandList = new CommandList();
        captureBack(commandList);
        return commandList;
    }
    
    static void sendCommands(CommandList commandList) throws IOException {
        for (Command command : commandList) sendCommand(command);
    }
    
    static int sendCommand(Command command) throws IOException {
        log.printf("Sending command: \"%s\"%n", command.getUnescapedCommandString());
        String urlString = String.format("http://debatedecide.fit.edu/robot.php?o=369&m=Y&p=%s&c=%s", password, command.getCommandString());
        URL url = new URL(urlString);
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        int responseCode = connection.getResponseCode();
        log.printf("HTTP Response Code: %d%n", responseCode);
        return responseCode;
    }
    
    static void downloadImages(CommandList commandList) throws MalformedURLException, IOException {
        for (Command command : commandList) {
            if (command instanceof CaptureCommand) {
                CaptureCommand captureCommand = (CaptureCommand)command;
                downloadImage(captureCommand.id);
            }
        }
    }
    
    static void downloadImage(int imageId) throws MalformedURLException, IOException {
        String fileName = String.format("%d.bmp", imageId);
        log.printf("Downloading Image #%d.%n", imageId);
        downloadFile("http://debatedecide.fit.edu/robot/" + fileName, "images/" + fileName);
    }
    
    //Credit to Holger of http://stackoverflow.com/questions/18872611/download-file-from-server-in-java
    //for the main downloading logic; now somewhat modified
    static void downloadFile(String sourceUrlString, String destinationPathString) throws MalformedURLException, IOException {
        URL url = new URL(sourceUrlString);
        InputStream inputStream = url.openStream();
        ReadableByteChannel inputChannel = Channels.newChannel(inputStream);
        
        try(FileOutputStream outputStream = new FileOutputStream(destinationPathString)) {
            FileChannel outputChannel = outputStream.getChannel();
            outputChannel.transferFrom(inputChannel, 0, Long.MAX_VALUE);
        }
    }
    
    static void deleteImages() throws IOException {
        log.println("Deleting images from the server.");
        sendCommand(CaptureCommand.DELETE_COMMAND);
    }
}
