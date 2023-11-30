// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class CustomDrive extends CommandBase {

  private static final double MAX_HEADING = 1;
  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY, heading;
  private double customHeading, minOmegaRadiansPerSecond;
  private SlewRateLimiter filter;

  /**
   * Used to drive a swerve robot in full field-centric mode. vX and vY supply
   * translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and
   * headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be
   * converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.
   *                Should be in the range -1 to 1 with
   *                deadband already accounted for. Positive X is away from the
   *                alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.
   *                Should be in the range -1 to 1 with
   *                deadband already accounted for. Positive Y is towards the left
   *                wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  public CustomDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
      DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    
    // Creates a SlewRateLimiter that limits the rate of change of the signal to X units per second
    filter = new SlewRateLimiter(Constants.Drivebase.ROTATION_RATE_PER_SEC);
    customHeading = 0;

    if(!RobotBase.isSimulation()) {
      minOmegaRadiansPerSecond = Constants.Drivebase.MIN_OMEGA_RADIANS_PER_SECOND;
    }
    else {
      minOmegaRadiansPerSecond = 0.001;
    }

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    double currentHeading = swerve.getHeading().getDegrees();
    currentHeading = currentHeading % 360;
    if(currentHeading < -180) {
      currentHeading = currentHeading + 360;
    }
    else if(currentHeading > 180) {
      currentHeading = currentHeading - 360;
    }
    filter.reset(currentHeading / 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(heading.getAsDouble() < -swerve.getSwerveController().config.angleJoyStickRadiusDeadband) {
      if(customHeading >= MAX_HEADING) {
        filter.reset(-MAX_HEADING);
      }
      customHeading = filter.calculate(MAX_HEADING);
    }
    else if(heading.getAsDouble() > swerve.getSwerveController().config.angleJoyStickRadiusDeadband) {
      if(customHeading <= -MAX_HEADING) {
        filter.reset(MAX_HEADING);
      }
      customHeading = filter.calculate(-MAX_HEADING);
    }
    else {
      /*This is needed to make sure the time used in the filter is always up to date,
      otherwise there will be a large jump in the value when the joystick commands a rotation */
      filter.reset(customHeading);
    }

    SmartDashboard.putNumber("A Custom Heading", customHeading);

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(-vX.getAsDouble(), -vY.getAsDouble(),
        new Rotation2d(customHeading * Math.PI));

    ChassisSpeeds customDesiredSpeed;
    double customOmega;

    if(Math.abs(desiredSpeeds.omegaRadiansPerSecond) < minOmegaRadiansPerSecond) {
      customDesiredSpeed = swerve.getTargetSpeeds(-vX.getAsDouble(), -vY.getAsDouble(), swerve.getHeading());
      customOmega = 0;
    }
    else {
      customDesiredSpeed = desiredSpeeds;
      customOmega = desiredSpeeds.omegaRadiansPerSecond;
    }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(customDesiredSpeed);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, customOmega, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
