// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.commands.ArmStuff.SimpleMoveToPickup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAtDouble extends ParallelCommandGroup {
  /** Creates a new AlignToDouble. */
  public IntakeAtDouble(Vision vision, Drivetrain drivetrain, Arm arm, Wrist wrist, Intake intake,
      Supplier<PieceType> piece,
      DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier rightX, Supplier<Boolean> isQuickTurn) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AlignToDouble(vision, drivetrain, leftY, rightY, rightX, isQuickTurn));
  }
}
