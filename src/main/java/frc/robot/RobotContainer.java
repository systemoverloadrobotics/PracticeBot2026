// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                           // second max angular
                                                                                           // velocity

  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {

    drivetrain.resetPose(new Pose2d(new Translation2d(0.0, 0.0), drivetrain.getRotation3d().toRotation2d()));

    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(Math.pow(joystick.getLeftY(), 3) * MaxSpeed) // Drive
            .withVelocityY(Math.pow(joystick.getLeftX(), 3) * MaxSpeed) // Drive
            .withRotationalRate(Math.pow(-joystick.getRightX(), 3) * MaxSpeed) // Drive
        ));

  }

}
