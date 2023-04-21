// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TiltWristDownAndStow extends SequentialCommandGroup {
  /** Creates a new TiltWristDownAndScore. */
  public TiltWristDownAndStow(Arm arm, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(//intake.SpinIntakeCommand(() -> PieceType.CUBE, true),
          new InstantCommand(() -> wrist.setPosition(wrist.getWristPosition()-16, () -> arm.getArmAngle())), 
          wrist.WaitUntilWristInPosition(), 
          new Stow(arm, wrist, intake));
  }
}
