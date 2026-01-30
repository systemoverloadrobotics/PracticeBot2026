// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Vision {
    public static final String LEFT_CAMERA = "LeftCamera";
    public static final String RIGHT_CAMERA = "RightCamera";

    public static final Transform3d LEFT_CAMERA_TO_SHOOTER = new Transform3d(
      new Translation3d(
        Inches.of(-17.5),
        Inches.of(-3),
        Inches.of(0.45)
      ),
      new Rotation3d(
        Degrees.of(0),
        Degrees.of(-30),
        Degrees.of(0)
      )
    );
    public static final Transform3d RIGHT_CAMERA_TO_SHOOTER = new Transform3d(
      new Translation3d(
        Inches.of(-17.5),
        Inches.of(-3),
        Inches.of(0.45)
      ),
      new Rotation3d(
        Degrees.of(0),
        Degrees.of(-30),
        Degrees.of(0)
      )
    );

    public static final Transform3d SHOOTER_TO_LEFT_CAMERA = LEFT_CAMERA_TO_SHOOTER.inverse();

    public static final Transform3d SHOOTER_TO_RIGHT_CAMERA = RIGHT_CAMERA_TO_SHOOTER.inverse();

    public static final int BLUE_ALLIANCE_HUB_TAG = 26;
    public static final int RED_ALLIANCE_HUB_TAG = 10; // TODO: Update Tag ID

    public static final Transform2d BLUE_ALLIANCE_TAG_TO_HUB = new Transform2d(
        Feet.of(1),
        Feet.of(0),
        new Rotation2d(Degrees.of(0))
    );

    public static final Transform2d RED_ALLIANCE_TAG_TO_HUB = new Transform2d(
        Feet.of(-1),
        Feet.of(0),
        new Rotation2d(Degrees.of(0))
    );
  }
}
