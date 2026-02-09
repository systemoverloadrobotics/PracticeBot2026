package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** 
 * Command for turning robot 180 degrees in place.
 */
public class TurnAround180 extends Command {
  //Tracks if the command is already running, if it is then it cancels itself to prevent multiple instances
  //of the command from running at the same time
  private static boolean isRunning = false; 

  private CommandSwerveDrivetrain drivetrain;
  private Rotation2d targetRotation;
  
  public TurnAround180(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    if (isRunning) {
      cancel();
      return;
    }
  
    Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
    this.targetRotation = currentRotation.plus(Rotation2d.fromDegrees(180));
  }

  @Override
  public void execute() {
    SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle()
      .withVelocityX(0)
      .withVelocityY(0)
      .withTargetDirection(targetRotation);

    this.drivetrain.setControl(request);
  }

  @Override
  public boolean isFinished() {
    Rotation2d currentRot = drivetrain.getState().Pose.getRotation();
    double angleDifference = currentRot.getDegrees() - targetRotation.getDegrees();
    angleDifference  = Math.IEEEremainder(angleDifference, 360);
    return (Math.abs(angleDifference) <= 1);
  }

  @Override
  public void end(boolean interrupted) {
    isRunning = false;
    drivetrain.idle();
  }

}
