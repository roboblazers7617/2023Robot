// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleMoveToPickup extends SequentialCommandGroup {
  /** Creates a new SimpleMoveArmToPosition. */
  public SimpleMoveToPickup(Arm arm, Intake intake, Supplier<PieceType> piece, PickupLocation location) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(arm.moveToHeldPositionCommand(location), intake.holdCommand()),
        new ParallelCommandGroup(intake.moveToPositionCommand(location, piece), arm.holdCommand()));

  }
}
