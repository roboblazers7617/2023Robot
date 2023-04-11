// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateMachine;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.StateConstants.GenericPosition;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleScore extends SequentialCommandGroup {
  /** Creates a new SimplePickup. */
  public SimpleScore(StateMachine machine, Intake intake, Supplier<PieceType> piece, Supplier<GenericPosition> position, boolean waitToStow) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(machine.changeStateCommand(piece, position), new OutakePiece(intake, piece), new Stow(machine, intake, waitToStow));
  }
}