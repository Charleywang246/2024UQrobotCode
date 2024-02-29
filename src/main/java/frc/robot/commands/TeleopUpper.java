package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.robotState;
import frc.robot.subsystems.UpperSub;

public class TeleopUpper extends Command{
    
    private final UpperSub sub;

    private final XboxController controller;

    private robotState state;

    private double elbowAngle;
    private double intakeSpeed;
    private double shooterSpeed;

    private boolean tele = false;

    private boolean ground = false;
    private boolean amp = false;
    private boolean speaker = false;

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP, 
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    private final PID shooterPID = new PID(
        UpperConstants.shooterKP, 
        UpperConstants.shooterKI,
        UpperConstants.shooterKD,
        UpperConstants.shooteriWindup,
        UpperConstants.shooteriLimit
    );

    public TeleopUpper(UpperSub sub, XboxController controller) {
        this.sub = sub;
        this.controller = controller;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        state = robotState.DEFAULT;
    }

    @Override
    public void execute() {

        switch (state) {
            case DEFAULT:
                elbowAngle = UpperConstants.ELBOW_DEFAULT_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                break;
            case GROUND:
                elbowAngle = UpperConstants.ELBOW_GROUND_POS;
                intakeSpeed = UpperConstants.INTAKE_GROUND_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_GROUND_SPEED;
                break;
            case AMP:
                elbowAngle = UpperConstants.ELBOW_AMP_POS;
                intakeSpeed = 0;
                shooterSpeed = 0;
                break;
            case SPEAKER:
                elbowAngle = UpperConstants.ELBOW_SPEAKER_POS;
                intakeSpeed = 0;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                break;
            case SHOOT:
                intakeSpeed = UpperConstants.INTAKE_SHOOT_SPEED;
                shooterSpeed = UpperConstants.SHOOTER_SHOOT_SPEED;
                break;
        }

        if(tele) {
            if(controller.getRightTriggerAxis() > 0.05) sub.setElbow(controller.getRightTriggerAxis());
            else if(controller.getLeftTriggerAxis() > 0.05) sub.setElbow(-controller.getLeftTriggerAxis());
            else sub.setElbow(0);

            if(controller.getXButton()) {
                sub.setIntake(-0.8);
                sub.setShooter(-1);
            }else {
                sub.setIntake(0);
                sub.setShooter(0);
            }
        }else {
            if(controller.getYButtonPressed()) {ground = !ground;amp=false;speaker=false;}
            if(controller.getXButtonPressed()) {amp = !amp;ground=false;speaker=false;}
            if(controller.getAButtonPressed()) {speaker = !speaker;ground=false;amp=false;} 
            if(controller.getRightTriggerAxis() > 0.1) {ground=false;amp=false;speaker=false;}

            if(ground) state = robotState.GROUND;
            else if(amp) state = robotState.AMP;
            else if(speaker) state = robotState.SPEAKER;
            else if(controller.getRightTriggerAxis() > 0.1) state = robotState.SHOOT;
            else state = robotState.DEFAULT;

            sub.setElbow(-elbowPID.calculate(elbowAngle - sub.getElbowRotation()));
            sub.setShooter(shooterSpeed);
            sub.setIntake(intakeSpeed);
        }

        if(controller.getLeftBumperPressed()) tele = !tele; 
        SmartDashboard.putString("robotState", state.toString());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
