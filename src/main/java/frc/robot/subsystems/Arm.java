// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
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
  private CANSparkMax shoulderMotor = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax shoulderMotorFollower = new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController controller;
  private SparkMaxPIDController controllerFollower;
  private RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();
  private RelativeEncoder shoulderFollowerEncoder = shoulderMotorFollower.getEncoder();
  private DoubleSolenoid leftPiston;
  private DoubleSolenoid rightPiston;
  private Pnuematics pneumatics;
  private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);

  //AnalogPotentiometer shoulderAngle = new AnalogPotentiometer(ArmConstants.SHOULDER_POTENTIOMETER_PORT, ArmConstants.SHOULDER_POTENTIOMETER_RANGE, ArmConstants.SHOULDER_POTENTIOMETER_OFFSET);

  private DigitalInput isArmStowed = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

  private Timer time = new Timer();

  private double dt, lastTime;

  private double setpoint = -54;

  public Arm(Pnuematics pnuematics) {
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotorFollower.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kBrake); 
    shoulderMotorFollower.setIdleMode(IdleMode.kBrake); 
   
    shoulderMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT); 
    shoulderMotorFollower.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);


    shoulderMotorFollower.follow(shoulderMotor, true);

    shoulderEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
    shoulderEncoder.setVelocityConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR/60.0);
    shoulderEncoder.setPosition(ArmConstants.MINIMUM_SHOULDER_ANGLE);
    shoulderFollowerEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
    shoulderFollowerEncoder.setVelocityConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR/60.0);
    shoulderFollowerEncoder.setPosition(ArmConstants.MINIMUM_SHOULDER_ANGLE);
  


    controller = shoulderMotor.getPIDController();
    controllerFollower = shoulderMotorFollower.getPIDController();
  
    controllerFollower.setP(ArmConstants.KP);
    controllerFollower.setI(ArmConstants.KI);
    controllerFollower.setD(ArmConstants.KD);
    controllerFollower.setOutputRange(ArmConstants.MAX_SPEED_DOWNWARD, ArmConstants.MAX_SPEED_UPWARD);

    
    controller.setP(ArmConstants.KP);
    controller.setI(ArmConstants.KI);
    controller.setD(ArmConstants.KD);
    controller.setOutputRange(ArmConstants.MAX_SPEED_DOWNWARD, ArmConstants.MAX_SPEED_UPWARD);
    
    this.pneumatics = pnuematics;
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
    //System.out.println("velocity " + shoulderEncoder.getVelocity());    
    
    if (getShoulderAngle() > ( ArmConstants.MINIMUM_SHOULDER_ANGLE + 4 )&& setpoint == ArmPositions.STOW.getShoulderAngle())
      shoulderMotor.set(0);
    else if (getShoulderAngle() < ( ArmConstants.MINIMUM_SHOULDER_ANGLE + 4 ) && setpoint == ArmPositions.STOW.getShoulderAngle()){
      controller.setReference(ArmPositions.STOW.getShoulderAngle(), CANSparkMax.ControlType.kPosition, 0,
      feedforward.calculate(Units.degreesToRadians(ArmPositions.STOW.getShoulderAngle()), 0));
    }
    
  }

  public void turnOnBrakes(Boolean isBraked)
  {
    if (isBraked)
    {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderMotorFollower.setIdleMode(IdleMode.kBrake);
    }
    else
    {
      shoulderMotor.setIdleMode(IdleMode.kCoast);
      shoulderMotorFollower.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public ArmPositions evalPickupPosition(Supplier<PickupLocation> location) {
    if (location.get().equals(PickupLocation.FLOOR))
      return ArmPositions.FLOOR_PICKUP_CONE;
    else if (location.get().equals(PickupLocation.DOUBLE))
      return ArmPositions.STATION_PICKUP;
    else
      return ArmPositions.STOW;
  }

  public ArmPositions evalScorePosition(Supplier<ScoreLevel> level) {
    if (level.get().equals(ScoreLevel.LEVEL_1))
      return ArmPositions.LEVEL_1_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_2))
      return ArmPositions.LEVEL_2_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_3))
      return ArmPositions.LEVEL_3_CONE;
    else
      return ArmPositions.STOW;
  }

  public void setPosition(double positionDegrees) {
    setpoint = Math.min(positionDegrees, ArmConstants.MAX_SHOULDER_ANGLE);
    setpoint = Math.max(setpoint, ArmConstants.MINIMUM_SHOULDER_ANGLE);
    controller.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(setpoint), 0));
    controllerFollower.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(setpoint), 0));
  }

  public void setPosition(ArmPositions position) {
    setPosition(position.getShoulderAngle());
  }

  public void setVelocity(double velocityDegreesPerSec){
    setpoint = setpoint + velocityDegreesPerSec*dt;
    setpoint = Math.min(setpoint, ArmConstants.MAX_SHOULDER_ANGLE);
    setpoint = Math.max(setpoint, ArmConstants.MINIMUM_SHOULDER_ANGLE);
    controller.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(setpoint), Units.degreesToRadians(velocityDegreesPerSec)));
    //System.out.println("Setpoint: " + setpoint);
  }

  public Command actuateSuperstructureCommandPickup(Supplier<PickupLocation> location) {
    return Commands.runOnce(() -> actuateSuperstructure(evalPickupPosition(location).getPistonPosition()), this);
  }

  public Command actuateSuperstructureCommandScore(Supplier<ScoreLevel> level) {
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

  public boolean atSetpoint(){
    return (Math.abs(getShoulderAngle() - (setpoint)) < (ArmConstants.POSITION_TOLERANCE));
  }

  public Command WaitUntilArmInPosition(){
    return Commands.waitUntil(() -> atSetpoint());
  }

  public void enableCompressor(boolean enableCompressor){
    if (enableCompressor)
    {
      pneumatics.enable();
    }
    else
    {
      pneumatics.disable();
    }
  }

  public double getShoulderMotorTemp(){
    return shoulderMotor.getMotorTemperature();
  }

public ArmPositions evalScoreLevel(ScoreLevel level) {
    return null;
}
}
