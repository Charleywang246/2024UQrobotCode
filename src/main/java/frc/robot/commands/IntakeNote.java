package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command{
        Timer t = new Timer();
    
    @Override
    public void initialize() {
        System.out.println("test auto init");
        t.restart();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("test auto finished");
    }

    @Override
    public boolean isFinished() {
        if(t.get() > 1) return true;
        else return false;
    }
}
