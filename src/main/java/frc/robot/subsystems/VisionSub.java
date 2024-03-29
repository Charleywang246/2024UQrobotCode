package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSub extends SubsystemBase{
    
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public double getTx() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTv() {
        return table.getEntry("tv").getDouble(0);
    }

    public double getTa() {
        return table.getEntry("ta").getDouble(0);
    }

    public boolean hasTarget() {
        return getTv() < 1.0 ? false : true;
    }

    public double[] getRobotPose() {
        return table.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double getRobotX() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[0];
    }

    public double getRobotY() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[1];
    }

    public double getRobotDEG() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[5];
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("RobotPose_X", getRobotX());
        SmartDashboard.putNumber("RobotPose_Y", getRobotY());
        SmartDashboard.putNumber("RobotPose_r", getRobotDEG());
    }
}