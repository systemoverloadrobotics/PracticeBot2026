package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;  
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** 
 * Command for turning robot 180 degrees in place.
 */
public class TurnAround180 extends Command {
  //Tracks if the command is already running, if it is then it cancels itself to prevent multiple instances
  //of the command from running at the same time
  private static boolean isRunning = false; 

  private CommandSwerveDrivetrain drivetrain;
  private Rotation2d targetRotation;

  private  CommandXboxController controller;
  
  public TurnAround180(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.controller =  controller;
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
    double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    SwerveRequest drive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(maxSpeed * 0.025).withRotationalDeadband(maxAngularRate * 0.025)
      .withTargetDirection(targetRotation)
      .withVelocityX(Math.pow(controller.getLeftY(), 3) * maxSpeed)
      .withVelocityY(Math.pow(controller.getLeftX(), 3) * maxSpeed);

    this.drivetrain.setControl(drive);
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
  }

}
