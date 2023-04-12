// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ArmConstants.WristConstants;
import frc.robot.Constants.ArmConstants.StateConstants.GenericPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.States.SetVelocitiesState;
import frc.robot.subsystems.Arm.States.WaitUntilAtSetpoint;
import frc.robot.subsystems.Arm.States.WaitUntilAtState;
import frc.robot.subsystems.Intake.Intake;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TiltWristDownAndStow extends SequentialCommandGroup {
  /** Creates a new TiltWristDownAndScore. */
  public TiltWristDownAndStow(Arm arm, Intake intake, Supplier<PieceType> piece) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(intake.SpinIntakeCommand(piece, true), new InstantCommand(() -> arm.setPosition(arm.getShoulderAngle(), arm.getWristAngle()-10), arm), new WaitUntilAtSetpoint(arm), new Stow(arm, intake, false));
  }
}
