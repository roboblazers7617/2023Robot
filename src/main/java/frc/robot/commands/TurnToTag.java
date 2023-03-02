// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTag extends PIDCommand {
  /** Creates a new TurnToTag. */
  public TurnToTag(Vision vision, Drivetrain mDrivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new PIDController(
            // The PID gains
            DrivetrainConstants.KP_ROT,
            DrivetrainConstants.KI_ROT,
            DrivetrainConstants.KD_ROT),
            // The motion profile constraints
        // This should return the measurement
        vision::getBestTagYaw,
        // This should return the goal (can also be a constant)
        () -> 0,
        // This uses the output
        (output) -> {
          // Use the output (and setpoint, if desired) here
              //TODO: Lukas. (High) Not sure should use KS_ROT here. KS_ROT. Use SIMPLE_FF_ANGULAR constant?
              //TODO: Lukas. Please move magic numbers into constants
          mDrivetrain.arcadeDrive(0.0,MathUtil.clamp(output+Math.copySign(DrivetrainConstants.KS_ROT, output), -.5, .5));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(mDrivetrain);
    getController().enableContinuousInput(-180, 180);
    //TODO: Lukas (High) please move to constants
    getController().setTolerance(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
