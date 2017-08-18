//Author: Roger Ballard
//Class: Robotics & AI, Spring 2017

import java.io.PrintStream;

public class Logger {
    PrintStream out;
    boolean doLogging;
    
    public Logger(PrintStream out, boolean doLogging) {
        this.out = out;
        this.doLogging = doLogging;
    }
    
    public void print(Object obj) {
        if (doLogging) out.print(obj);
    }
    
    public void println(Object x) {
        if (doLogging) out.println(x);
    }
    
    public void printf(String format, Object... args) {
        if (doLogging) out.printf(format, args);
    }
}
