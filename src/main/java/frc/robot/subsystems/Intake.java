// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.WristConstants.IntakeConstants;
import frc.robot.Constants.WristConstants.IntakeConstants.IntakeDirection;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
//  ColorSensorV3 cubeSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final DigitalInput isIntakeStored = new DigitalInput(IntakeConstants.INTAKE_LIMIT_SWITCH_ID);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeEncoder.setVelocityConversionFactor(IntakeConstants.INTAKE_ENCODER_CONVERSION_FACTOR / 60);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isHoldingGamePiece() {
  
        if ((intakeMotor.getOutputCurrent() >= IntakeConstants.CONE_CURRENT_LIMIT ) /*|| (cubeSensor.getProximity() >= IntakeConstants.CUBE_SENSOR_LIMIT) */){
      return true;
    }
    else {
      return false;
    }

  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakeSpeed(PieceType piece, boolean isIntaking)
  {
    setIntakeSpeed(evalPieceIntake(piece, isIntaking).speed());
  }

  public Command SpinIntakeCommand(Supplier<PieceType> piece, boolean isIntaking) {
    return Commands.startEnd((() -> this.setIntakeSpeed(evalPieceIntake(piece.get(), isIntaking).speed())),
        (() -> this.setIntakeSpeed(IntakeDirection.STOP.speed())), this);
    
  }


  private IntakeDirection evalPieceIntake(PieceType piece, boolean isIntaking) {
    if (piece.equals(PieceType.CONE) && isIntaking && !isHoldingGamePiece())
      return IntakeDirection.PICK_CONE;
    else if (piece.equals(PieceType.CUBE) && isIntaking && !isHoldingGamePiece())
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

  public double getCurent()
  {
    return intakeMotor.getOutputCurrent();
  }

  public double getEncoderPosition(){
    return intakeEncoder.getPosition();
  }

  public double getMotorTemperature()
  {
    return intakeMotor.getMotorTemperature();
  }

}
