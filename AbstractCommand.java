//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

public abstract class AbstractCommand implements Command {
    @Override
    public String getCommandString() {
        return Utilities.htmlEscape(getUnescapedCommandString());
    }
}
