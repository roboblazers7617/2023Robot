// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.WristPosition;

public class Wrist extends SubsystemBase {
  private final CANSparkMax wristMotor = new CANSparkMax(IntakeConstants.WRIST_CAN_ID, MotorType.kBrushless);

  private final ArmFeedforward wristFeedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG,
      IntakeConstants.WRIST_KV);

  private final SparkMaxPIDController wristController = wristMotor.getPIDController();

  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  //TODO: private final DigitalInput isStowed = new DigitalInput(IntakeConstants.WRIST_LIMIT_SWITCH_CHANEL);

  private Timer time = new Timer();

  private double dt, lastTime;
  private double setpoint = WristPosition.STOW.angle();

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristEncoder.setPositionConversionFactor(IntakeConstants.WRIST_ENCODER_CONVERSION_FACTOR);
    wristEncoder.setVelocityConversionFactor(IntakeConstants.WRIST_ENCODER_CONVERSION_FACTOR / 60);
    wristEncoder.setPosition(IntakeConstants.MAX_WRIST_ANGLE);

    wristController.setP(IntakeConstants.WRIST_KP);
    wristController.setI(IntakeConstants.WRIST_KI);
    wristController.setD(IntakeConstants.WRIST_KD);
    wristController.setOutputRange(IntakeConstants.MAX_DOWNWARD_WRIST_SPEED, IntakeConstants.MAX_UPWARD_WRIST_SPEED);
    wristController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        wristFeedforward.calculate(Units.degreesToRadians(setpoint), 0));
    
        time.reset();
        time.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dt = time.get() - lastTime;
    lastTime = time.get();

   /* if(isStowed.get())
      wristEncoder.setPosition(WristPosition.STOW.angle());*/
  }

  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  public double getWristVelocity() {
    return wristEncoder.getVelocity();
  }

  public void setPosition(double position) {
    setpoint = Math.min(position, IntakeConstants.MAX_WRIST_ANGLE);
    setpoint = Math.max(setpoint, IntakeConstants.MIN_WRIST_ANGLE);
    wristController.setReference(position, CANSparkMax.ControlType.kPosition, 0,
        wristFeedforward.calculate(Units.degreesToRadians(position), 0));
  }

  public void setPosition(WristPosition position) {
    double setpoint = Math.min(position.angle(), IntakeConstants.MAX_WRIST_ANGLE);
    setpoint = Math.max(setpoint, IntakeConstants.MIN_WRIST_ANGLE);
    wristController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        wristFeedforward.calculate(Units.degreesToRadians(setpoint), 0));
  }

  public void setVelocity(double velocity){
    setpoint = setpoint + velocity*dt;
    setpoint = Math.min(setpoint, IntakeConstants.MAX_WRIST_ANGLE);
    setpoint = Math.max(setpoint, IntakeConstants.MIN_WRIST_ANGLE);
    wristController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        wristFeedforward.calculate(Units.degreesToRadians(setpoint), Units.degreesToRadians(velocity)));
    System.out.println("dt:" + dt);
    System.out.println("setpoint:" + setpoint);
  }

  public double getWristMotorTemp(){
    return wristMotor.getMotorTemperature();
  }
}
