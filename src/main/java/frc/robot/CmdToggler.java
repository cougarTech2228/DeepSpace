import edu.wpi.first.wpilibj.command.Command;

public class CmdToggler {
    Toggler toggle;
    Command command;
    boolean cmdRun;
    boolean cmdEnd;

    public CmdToggler(Command c) {
        command = c;
        toggle = new Toggler(2, true);
        cmdRun = false;
        cmdEnd = false;
    }
    public void runCommand(boolean button) {
        int i = toggle.toggle(button);
        if(i == 1) {
            if(!cmdRun) {
                command.start();
            }
            cmdRun = true;
            cmdEnd = false;
        }
        if(i == 0) {
            if(!cmdEnd) {
                command.start();
            }
            cmdEnd = true;
            cmdRun = false;
        }      
    }
}