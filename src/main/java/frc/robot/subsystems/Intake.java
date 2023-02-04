// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeDirection;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax( IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax wristMotor = new CANSparkMax(IntakeConstants.WRIST_CAN_ID, MotorType.kBrushless);
  private final AnalogPotentiometer wristPotentiometer = new AnalogPotentiometer(IntakeConstants.POT_CHANEL, IntakeConstants.WRIST_POT_SCALE);
  private final DigitalInput isIntakeStored = new DigitalInput(IntakeConstants.WRIST_LIMIT_SWITCH_CHANEL);
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
  public double getWristAngle(){
    return wristPotentiometer.get();
  }
  public void setWristSpeed (double speed) {
    if((speed < 0.0) && (isStored() == false)) {
      wristMotor.set(MathUtil.clamp(speed,-IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_SPEED));
    } else if ((speed > 0.0) && (wrisPotentiometer.get()<= IntakeConstants.MAX_WRIST_ANGLE)){
      wristMotor.set(MathUtil.clamp(speed,-IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_SPEED));
    } else {
      wristMotor.set(0.0);
    }
  }
  public boolean isStored() {
    return isIntakeStored.get();
  }
  public void setIntakeSpeed (double speed) {
    intakeMotor.set(speed);
  }
  public Command SpinIntake(IntakeDirection direction){
    double speed = direction.speed();
    return new RunCommand(()-> this.setIntakeSpeed(speed), this);
  }
  public Command moveToPosition(IntakeConstants.WristPostion position){
    ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG, IntakeConstants.WRIST_KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
      new ProfiledPIDController(IntakeConstants.WRIST_KP, 
          IntakeConstants.WRIST_KI, IntakeConstants.WRIST_KD, 
              new TrapezoidProfile.Constraints(IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_ACCEL)),
       ()->this.getWristAngle(),
        direction.angle(),
     (output, setpoint) -> {
          this.setWristSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));},
      this);

    command.getController().setTolerance(IntakeConstants.WRIST_ANGLE_TOLERANCE);

    return command; 
    
  }

}
