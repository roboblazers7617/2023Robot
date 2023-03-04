// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleMoveToPickup extends SequentialCommandGroup {
  /** Creates a new SimpleMoveArmToPosition. */
  public SimpleMoveToPickup(Arm arm, Wrist wrist, Supplier<PieceType> piece, Supplier<PickupLocation> location) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> arm.setPosition(arm.evalPickupPosition(location)), arm),
        arm.WaitUntilArmInPosition(),
        arm.actuateSuperstructureCommandPickup(location),
        new InstantCommand(() -> wrist.setPosition(wrist.evalPickupLocation(location, piece), arm::getShoulderAngle), wrist),
        wrist.WaitUntilWristInPosition());

  }
}
