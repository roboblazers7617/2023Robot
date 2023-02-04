// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax( IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax wristMotor = new CANSparkMax(IntakeConstants.WRIST_CAN_ID, MotorType.kBrushless);
  private final AnalogPotentiometer wrisPotentiometer = new AnalogPotentiometer(IntakeConstants.POT_CHANEL, IntakeConstants.WRIST_POT_SCALE);
  private final DigitalInput isStored = new DigitalInput(IntakeConstants.WRIST_LIMIT_SWITCH_CHANEL);
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    wristMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    wristMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setWristSpeed (double speed) {
    if((speed < 0.0) && (isStored == false)) {
      wristMotor.set(speed);
    }
  }
  public boolean isIntakeStored() {
    return isStored;
  }
}
