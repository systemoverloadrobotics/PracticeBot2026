package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PointToHub extends Command {

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                             // second max angular
                                                                                             // velocity

    PhotonCamera leftCamera = new PhotonCamera(Constants.Vision.LEFT_CAMERA);
    PhotonCamera rightCamera = new PhotonCamera(Constants.Vision.RIGHT_CAMERA);

    ProfiledPIDController yawController;

    CommandSwerveDrivetrain drivetrain;
    CommandXboxController controller;

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    Transform3d tagToHub;

    // Align to this tag
    public int tag;

    public double yawOutput;

    public PointToHub(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {

        this.controller = controller;

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            end(false);
            return;
        }

        if (alliance.get() == Alliance.Blue) {
            tag = Constants.Vision.BLUE_ALLIANCE_HUB_TAG;
            var t = Constants.Vision.BLUE_ALLIANCE_TAG_TO_HUB;
            this.tagToHub = new Transform3d(
                    new Translation3d(
                            t.getX(),
                            t.getY(),
                            0.0),
                    new Rotation3d(
                            0.0,
                            0.0,
                            t.getRotation().getRadians()));
        } else {
            tag = Constants.Vision.RED_ALLIANCE_HUB_TAG;
            var t = Constants.Vision.RED_ALLIANCE_TAG_TO_HUB;
            this.tagToHub = new Transform3d(
                    new Translation3d(
                            t.getX(),
                            t.getY(),
                            0.0),
                    new Rotation3d(
                            0.0,
                            0.0,
                            t.getRotation().getRadians()));
        }

        this.drivetrain = drivetrain;

        TrapezoidProfile.Constraints yawConstraints = new TrapezoidProfile.Constraints(1.33 * 0.75, 1.5 * 0.75);
        yawController = new ProfiledPIDController(7.0, 0, 0, yawConstraints);

        yawController.setTolerance(0.05 * 3);

        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var driveState = drivetrain.getState();
        var drivePose = driveState.Pose;
        var driveSpeeds = driveState.Speeds;

        yawController.reset(drivePose.getRotation().getRadians(), driveSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        var leftResults = leftCamera.getAllUnreadResults();
        var rightResults = rightCamera.getAllUnreadResults();

        if (leftResults.isEmpty() && rightResults.isEmpty()) {
            return;
        }

        Transform3d shooterToHub = null;

        if (!leftResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = leftResults.get(leftResults.size() - 1);

            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {

                    if (target.getFiducialId() != tag) {
                        continue;
                    }

                    Transform3d cameraToTag = target.getBestCameraToTarget();
                    Transform3d shooterToCamera = Constants.Vision.SHOOTER_TO_LEFT_CAMERA;

                    shooterToHub = shooterToCamera.plus(cameraToTag).plus(tagToHub);
                    break;
                }
            }
        }

        if (shooterToHub == null && rightResults.isEmpty() == false) {
            var result = rightResults.get(rightResults.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == tag) {
                        Transform3d cameraToTag = target.getBestCameraToTarget();
                        Transform3d shooterToCamera = Constants.Vision.SHOOTER_TO_RIGHT_CAMERA;

                        shooterToHub = shooterToCamera.plus(cameraToTag).plus(tagToHub);
                        break;
                    }
                }
            }
        }


        if (shooterToHub == null) {
            control(this.yawOutput);
            return;
        }

        double x = shooterToHub.getTranslation().getX();
        double y = shooterToHub.getTranslation().getY();
        double targetYaw = Math.atan2(y, x);

        double currentYaw = drivetrain.getState().Pose.getRotation().getRadians();

        yawController.setGoal(targetYaw);
        this.yawOutput = yawController.calculate(currentYaw);

        control(this.yawOutput);
    }

    public void control(double yawOutput) {

        drive.withRotationalRate(yawOutput)
                .withVelocityX(Math.pow(controller.getLeftY(), 3) * MaxSpeed)
                .withVelocityY(Math.pow(controller.getLeftX(), 3) * MaxSpeed);

        drivetrain.setControl(
                drive);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.FieldCentric().withRotationalRate(0));

        super.end(interrupted);
    }
}