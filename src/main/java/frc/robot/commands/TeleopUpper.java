package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.UpperSub;

public class TeleopUpper extends Command{
    
    private final UpperSub sub;

    private final XboxController controller;
    
    private boolean tele = false;

    public TeleopUpper(UpperSub sub, XboxController controller) {
        this.sub = sub;
        this.controller = controller;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(controller.getYButtonPressed()) {
            sub.setState(UpperState.GROUND);
        }
        if(controller.getXButtonPressed()) {
            sub.setState(UpperState.AMP);
        }
        if(controller.getAButtonPressed()) {
            sub.setState(UpperState.SPEAKER);
        }
        if(controller.getRightTriggerAxis() > 0.02) {
            sub.setState(UpperState.SHOOT);
        }
        if(controller.getLeftBumper()) {
            sub.setState(UpperState.TELE);
            tele = !tele;
        }

        if(tele) {
            if(controller.getRightTriggerAxis() > 0.02) sub.setElbow(controller.getRightTriggerAxis());
            else if(controller.getLeftTriggerAxis() > 0.02) sub.setElbow(-controller.getLeftTriggerAxis());
            else sub.setElbow(0);

            if(controller.getXButton()) {
                sub.setIntake(-0.8);
                sub.setShooter(-1);
            }else {
                sub.setIntake(0);
                sub.setShooter(0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
