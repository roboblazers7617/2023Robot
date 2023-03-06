// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ScoreLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleMoveToScore extends SequentialCommandGroup {
  /** Creates a new SimpleMoveArmToPosition. */
  public SimpleMoveToScore(Arm arm, Wrist wrist, Supplier<ScoreLevel> level, Supplier<PieceType> piece) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(arm.intigratedMoveToScore(level, piece),
    new InstantCommand(() -> wrist.setPosition(wrist.evalScorePosition(level), arm::getArmAngle), wrist),
    wrist.WaitUntilWristInPosition());
  }
}
