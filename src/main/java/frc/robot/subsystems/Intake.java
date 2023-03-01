// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.WristConstants.IntakeConstants;
import frc.robot.Constants.WristConstants.IntakeConstants.IntakeDirection;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
  private final DigitalInput isHoldingCube = new DigitalInput(IntakeConstants.DISTANCE_SENSOR_CHANEL);
  private final DigitalInput isIntakeStored = new DigitalInput(IntakeConstants.INTAKE_LIMIT_SWITCH_ID);

  

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    //TODO: Marie. (High) Add velocity converstion factor for intake
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isHoldingGamePiece() {
    // TODO: Marie. (High) add in code for holding a cone or cube
    //return isHoldingCube.get();'
    return false;

  }

  //TODO: Marie. (High) Add code to check encoder value as the limit switch may not always be tripped with stowed
  public boolean isStored() {
    return isIntakeStored.get();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public Command SpinIntakeCommand(Supplier<PieceType> piece, boolean isIntaking) {

    return Commands.startEnd((() -> this.setIntakeSpeed(evalPieceIntake(piece.get(), isIntaking).speed())),
        (() -> this.setIntakeSpeed(IntakeDirection.STOP.speed())), this);
  }


  private IntakeDirection evalPieceIntake(PieceType piece, boolean isIntaking) {
    if (piece.equals(PieceType.CONE) && isIntaking)
      return IntakeDirection.PICK_CONE;
    else if (piece.equals(PieceType.CUBE) && isIntaking)
      return IntakeDirection.PICK_CUBE;
    if (piece.equals(PieceType.CONE) && !isIntaking)
      return IntakeDirection.PLACE_CONE;
    else if (piece.equals(PieceType.CUBE) && !isIntaking)
      return IntakeDirection.PLACE_CUBE;
    else
      return IntakeDirection.STOP;
  }

  public double getIntakeSpeed() {
    return intakeMotor.getEncoder().getVelocity();
  }

}
