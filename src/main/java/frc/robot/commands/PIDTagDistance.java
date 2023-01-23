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
public class PIDTagDistance extends PIDCommand {
  /** Creates a new distanceFromTag. */
  public PIDTagDistance(Vision vision, Drivetrain drivetrain, double distanceMeters) {
    super(
        // The controller that the command will use
        new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
        // This should return the measurement
        vision::getBestTagDistance,
        // This should return the setpoint (can also be a constant)
        () -> distanceMeters,
        // This uses the output
        (output) -> {
          // Use the output here
          if(vision.getBestTagDistance() > 0)
          drivetrain.arcadeDrive(MathUtil.clamp(-output, -0.5, .5), 0);
          else 
            drivetrain.arcadeDrive(0, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    getController().setTolerance(.03);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
