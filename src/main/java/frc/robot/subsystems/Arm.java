// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.ScoreLevel;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax shoulderMotor = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
  SparkMaxPIDController controller;
  RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();
  DoubleSolenoid leftPiston;
  DoubleSolenoid rightPiston;
  ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);

  AnalogPotentiometer shoulderAngle = new AnalogPotentiometer(ArmConstants.SHOULDER_POTENTIOMETER_PORT, ArmConstants.SHOULDER_POTENTIOMETER_RANGE, ArmConstants.SHOULDER_POTENTIOMETER_OFFSET);

  DigitalInput isArmStowed = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

  private Timer time = new Timer();

  private double dt, lastTime;

  private double setpoint;

  public Arm(Pnuematics pnuematics) {
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kCoast); // TODO: brake mode
    shoulderMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    controller = shoulderMotor.getPIDController();

    shoulderEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
    shoulderEncoder.setVelocityConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR/60.0);
    shoulderEncoder.setPosition(ArmConstants.MINIMUM_SHOULDER_ANGLE);

    controller.setP(ArmConstants.KP);
    controller.setI(ArmConstants.KI);
    controller.setD(ArmConstants.KD);
    controller.setOutputRange(ArmConstants.MAX_SPEED_DOWNWARD, ArmConstants.MAX_SPEED_UPWARD);

    leftPiston = pnuematics.getLeftArmPiston();
    rightPiston = pnuematics.getRightArmPiston();

    time.reset();
    time.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dt = time.get() -lastTime;
    lastTime = time.get();
  }

  public ArmPositions evalPickupPosition(PickupLocation location) {
    if (location.equals(PickupLocation.FLOOR))
      return ArmPositions.FLOOR_PICKUP;
    else if (location.equals(PickupLocation.DOUBLE))
      return ArmPositions.STATION_PICKUP;
    else
      return ArmPositions.STOW;
  }

  public ArmPositions evalScorePosition(ScoreLevel level) {
    if (level.equals(ScoreLevel.LEVEL_1))
      return ArmPositions.LEVEL_1;
    else if (level.equals(ScoreLevel.LEVEL_2))
      return ArmPositions.LEVEL_2;
    else if (level.equals(ScoreLevel.LEVEL_3))
      return ArmPositions.LEVEL_3;
    else
      return ArmPositions.STOW;
  }

  public void setPosition(double positionDegrees) {
    setpoint = Math.min(positionDegrees, ArmConstants.MAX_SHOULDER_ANGLE);
    setpoint = Math.max(setpoint, ArmConstants.MINIMUM_SHOULDER_ANGLE);
    controller.setReference(positionDegrees, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(setpoint), 0));
  }

  public void setPosition(ArmPositions position) {
     setpoint = Math.min(position.getShoulderAngle(), ArmConstants.MAX_SHOULDER_ANGLE);
    setpoint = Math.max(setpoint, ArmConstants.MINIMUM_SHOULDER_ANGLE);
    controller.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(setpoint), 0));
  }

  public void setVelocity(double velocityDegreesPerSec){
    setpoint = setpoint + velocityDegreesPerSec*dt;
    setpoint = Math.min(setpoint, ArmConstants.MAX_SHOULDER_ANGLE);
    setpoint = Math.max(setpoint, ArmConstants.MINIMUM_SHOULDER_ANGLE);
    controller.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(setpoint), Units.degreesToRadians(velocityDegreesPerSec)));
  }

  public Command actuateSuperstructureCommand(PickupLocation location) {
    return Commands.runOnce(() -> actuateSuperstructure(evalPickupPosition(location).getPistonPosition()), this);
  }

  public Command actuateSuperstructureCommand(ScoreLevel level) {
    return Commands.runOnce(() -> actuateSuperstructure(evalScorePosition(level).getPistonPosition()), this);
  }

  public Command actuateSuperstructureCommand(PnuematicPositions position) {
    return Commands.runOnce(() -> actuateSuperstructure(position), this);
  }

  public void actuateSuperstructure(PnuematicPositions positions) {
    leftPiston.set(positions.getValue());
    rightPiston.set(positions.getValue());
  }

  public Value getSuperstructureState() {
    return leftPiston.get();
  }

  public boolean isArmStowed() {
    return isArmStowed.get();
  }

  public double getShoulderAngle(){
    return shoulderEncoder.getPosition();
  }

  public boolean atSetpoint(){//TODO: check
    return (Math.abs(getShoulderAngle() - (setpoint)) < (ArmConstants.POSITION_TOLERANCE));
  }

  public Command WaitUntilArmInPosition(){
    return Commands.waitUntil(() -> atSetpoint());
  }
}
