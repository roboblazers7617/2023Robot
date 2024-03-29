// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;
import frc.robot.Constants.WristConstants.WristPosition;
import frc.robot.Constants.WristConstants.IntakeConstants.IntakeDirection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowAuton extends SequentialCommandGroup {
  /** Creates a new Stow. */
  public StowAuton(Arm arm, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> intake.setIntakeSpeed(IntakeDirection.STOP.speed()), intake),
        new InstantCommand(() -> wrist.setPosition(WristPosition.STOW, arm::getWrappedArmPosition), wrist),
        new InstantCommand(() -> arm.actuateSuperstructure(PnuematicPositions.RETRACTED)), new WaitCommand(.3),
        new InstantCommand(() -> arm.setPosition(ArmPositions.STOW), arm),
       arm.WaitUntilArmInPosition());
  }
}
