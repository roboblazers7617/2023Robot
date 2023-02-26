// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */


public Command PickPath(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle) {
  PathPlannerTrajectory test_path = PathPlanner.generatePath(
    new PathConstraints(.1, .1),
    new PathPoint(new Translation2d(x.getAsDouble(),y.getAsDouble()), new Rotation2d(angle.getAsDouble())),
   //new PathPoint(new Translation2d(12.75, 1), Rotation2d.fromDegrees(180)),
    new PathPoint(new Translation2d(14.61, 1.07), Rotation2d.fromDegrees(0)));

  SmartDashboard.putNumber("x", x.getAsDouble());
  SmartDashboard.putNumber("y", y.getAsDouble());
  SmartDashboard.putNumber("angle", angle.getAsDouble());

  PPRamseteCommand returnCommand = new PPRamseteCommand(
      test_path, 
      drivetrain::getPose2d, 
      new RamseteController(), 
      new SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV),
      drivetrain.getKinematics(),
      drivetrain::getWheelSpeeds,
      new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
      new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
      drivetrain::tankDriveVolts,
      false,
      drivetrain);
  return returnCommand;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
