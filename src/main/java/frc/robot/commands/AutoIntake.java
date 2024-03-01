package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants.UpperConstants;
import frc.robot.subsystems.UpperSub;

public class AutoIntake extends Command{

    UpperSub sub;

    Timer timer = new Timer();

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP,
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    public AutoIntake(UpperSub sub) {
        this.sub = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        System.out.println("Intake start");
    }

    @Override
    public void execute() {
        sub.setElbow(elbowPID.calculate(sub.getElbowRotation() - (-0.236328)));
        sub.setIntake(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        sub.setElbow(0);
        timer.stop();
        System.out.println("Intake end");
    }

    @Override
    public boolean isFinished() {
        if(timer.get()>10) return true;
        else return false;
    }
    
}
