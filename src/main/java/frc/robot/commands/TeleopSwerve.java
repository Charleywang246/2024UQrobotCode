package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(Swerve s_Swerve, XboxController controller) {
    this.s_Swerve = s_Swerve;
    this.driver = controller;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(driver.getLeftY(), Constants.
            Swerve.axisDeadBand));
    strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(driver.getLeftX(), Constants.Swerve.axisDeadBand));        
    rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(driver.getRightX() * 0.5, Constants.Swerve.axisDeadBand));

    if (driver.getBackButton()) s_Swerve.zeroGyro();

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, true,
        true);
  }
}