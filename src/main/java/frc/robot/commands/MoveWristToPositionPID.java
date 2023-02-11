// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveWristToPositionPID extends ProfiledPIDCommand {
  private Intake intake;

      
  public MoveWristToPositionPID(double targetAngle, Intake intake) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)),
        // This should return the measurement
        () -> intake.getWristAngle(),
        // This should return the goal (can also be a constant)
        targetAngle,
        // This uses the output
        (output, setpoint) -> { 
          // Use the output (and setpoint, if desired) here
          ArmFeedforward m_feedforward =
          new ArmFeedforward(
              0, 0,
              0, 0);
          intake.setWristSpeed(output + m_feedforward.calculate(setpoint.position, setpoint.velocity));
        });
        this.intake = intake;
        addRequirements(intake);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(1.0);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
