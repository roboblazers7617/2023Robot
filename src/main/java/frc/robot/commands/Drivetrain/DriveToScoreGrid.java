// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToScoreGrid extends SequentialCommandGroup {
  /** Creates a new TargetDrive. */
  public DriveToScoreGrid(Drivetrain drivetrain, DoubleSupplier LeftY, DoubleSupplier rightY, DoubleSupplier rightX, Supplier<Pose2d> targetPose, Alliance color) {

    addCommands(new DriveTillY(drivetrain, LeftY, rightY, rightX, targetPose),
              new FaceScoreLocation(drivetrain, color),
              new DriveForwardToScoreLocation(drivetrain, targetPose, color),
              new FaceScoreLocation(drivetrain, color)
     );
  }

  public DriveToScoreGrid(Drivetrain drivetrain, DoubleSupplier LeftY, DoubleSupplier rightY, DoubleSupplier rightX, Alliance color) {

    addCommands(new DriveTillY(drivetrain, LeftY, rightY, rightX),
              new FaceScoreLocation(drivetrain, color),
              new DriveForwardToScoreLocation(drivetrain, color),
              new FaceScoreLocation(drivetrain, color)
     );
  }
}
