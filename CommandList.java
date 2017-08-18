//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

import java.util.LinkedList;
import java.util.stream.Collectors;

public class CommandList extends LinkedList<Command> implements Command{
    private static final long serialVersionUID = 1L;
    
    @Override
    public String toString() {
        return String.join(String.format("%n"), this.stream().map(c -> c.getUnescapedCommandString()).collect(Collectors.toList()));
    }
    
    @Override
    public String getCommandString() {
        return Utilities.htmlEscape(getUnescapedCommandString());
    }

    @Override
    public String getUnescapedCommandString() {
        return toString().replace(String.format("%n"), " ");
    }
}
