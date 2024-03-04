package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class AutoAim extends Command{
    private final Swerve swerve;
    private final UpperSub upper;
    private final VisionSub vision;

    private final PID xyPID = new PID(0.03, 0, 0, 0, 0);
    private final PID zPID = new PID(0.001, 0, 0, 0, 0);

    private final PID elbowPID = new PID(
        UpperConstants.elbowKP, 
        UpperConstants.elbowKI,
        UpperConstants.elbowKD,
        UpperConstants.elbowiWindup,
        UpperConstants.elbowiLimit
    );

    private double xTarget = 0, yTarget = 0, zRotation = 0;

    public AutoAim(Swerve swerve, UpperSub upper, VisionSub vision) {
        this.swerve = swerve;
        this.upper = upper;
        this.vision = vision;
        addRequirements(swerve, upper, vision);
    }

    public void setPose(double x, double y) {
        xTarget = x;
        yTarget = y;
    }
    
    @Override
    public void initialize() {
        upper.setState(UpperState.TELE);
    }

    @Override
    public void execute() {

        Translation2d speaker = VisionConstants.Speaker_red;
        double distance = swerve.getPose().getTranslation().getDistance(speaker);

        if(vision.hasTarget()) {
            zRotation = Math.atan(
                (speaker.getX() - swerve.getPose().getX()) / (speaker.getY() - swerve.getPose().getY())
            );
            swerve.resetPose(new Pose2d(vision.getRobotX(), vision.getRobotY(), Rotation2d.fromDegrees(vision.getRobotDEG())));

        }

        swerve.drive(
            new Translation2d(xyPID.calculate(xTarget - swerve.getPose().getX()), xyPID.calculate(yTarget - swerve.getPose().getY())), 
            zPID.calculate(zRotation - swerve.getPose().getRotation().getRotations()), true, false
        );
        upper.setElbow(elbowPID.calculate(upper.calculateElbow(distance) - upper.getElbowRotation()));
        upper.setShooter(-1);
    }

    @Override
    public void end(boolean interrupted) {
        upper.setState(UpperState.SHOOT);
    }

    @Override
    public boolean isFinished() {
        if(
            Math.abs(xTarget - swerve.getPose().getX()) < 0.05 &&
            Math.abs(yTarget - swerve.getPose().getY()) < 0.05 &&
            Math.abs(zRotation - swerve.getPose().getRotation().getRotations()) < 0.005
        ) return true;
        else return false;
    }
}
