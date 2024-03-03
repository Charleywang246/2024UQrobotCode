package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.UpperSub;

public class TeleopUpper extends Command{
    
    private UpperSub sub;

    private final XboxController controller;

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

        if(UpperConstants.teleMode) {
            if(controller.getRightTriggerAxis() > 0.02) sub.setElbow(controller.getRightTriggerAxis());
            else if(controller.getLeftTriggerAxis() > 0.02) sub.setElbow(-controller.getLeftTriggerAxis());
            else sub.setElbow(0);

            if(controller.getXButton()) sub.setShooter(-1);
            else sub.setShooter(0);

            if(controller.getBButton()) sub.setIntake(1);
            else sub.setIntake(0);
        } else {
            if(controller.getRightTriggerAxis() > 0.02) {
                sub.setState(UpperState.SHOOT);
            } else {
                if(sub.getState() == UpperState.SHOOT) {
                    sub.setState(UpperState.DEFAULT);
                } else {
                    if(controller.getYButtonPressed()) sub.setState(UpperState.GROUND);
                    if(controller.getXButtonPressed()) sub.setState(UpperState.AMP);
                    if(controller.getAButtonPressed()) sub.setState(UpperState.SPEAKER);
                    if(controller.getStartButtonPressed()) sub.setState(UpperState.ENDGAME);
                }
            }
        }

        if(controller.getLeftBumperPressed()) {
            UpperConstants.teleMode = !UpperConstants.teleMode;
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
