// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class FaceScoreLocation extends CommandBase {
  /** Creates a new FaceScoreLocation. */
  private PIDController turnController = 
        new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
  private Drivetrain drivetrain;
  private Alliance color;
  private double targetAngle;
  private final double angleTolerance = 1.0;
  ShuffleboardTab shuffleboardTabTesting = Shuffleboard.getTab("drivetrain");

  public FaceScoreLocation(Drivetrain drivetrain, Alliance color) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.color = color;
    turnController.setTolerance(angleTolerance);
    turnController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (color == Alliance.Blue)
    {
      targetAngle = 180.0;
    }
    else
    {
      targetAngle = 0.0;
    }
   turnController.setSetpoint(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = turnController.calculate(drivetrain.getRotation2d().getDegrees());
    double simpleFF = Math.copySign(DrivetrainConstants.KS_ROT, output);
    drivetrain.arcadeDrive(0.0,MathUtil.clamp(output+ simpleFF, -DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_VELOCITY));
    System.out.println("**** Executing Turn: output is " + output + "    target angle is " + targetAngle + "  current angle is " + drivetrain.getRotation2d().getDegrees());
    System.out.println("Simple FF is " + simpleFF); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDone = false;
    double angle = Math.abs(drivetrain.getRotation2d().getDegrees());
    if ((angle >= (targetAngle - angleTolerance)) 
              &&  (angle <= (targetAngle + angleTolerance)))
    {
      isDone = true;
    }

    return isDone;
  }
}
