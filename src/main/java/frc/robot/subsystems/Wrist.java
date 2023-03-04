// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristPosition;

public class Wrist extends SubsystemBase {
  private final CANSparkMax wristMotor = new CANSparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);

  private final ArmFeedforward wristFeedforward = new ArmFeedforward(WristConstants.WRIST_KS, WristConstants.WRIST_KG,
      WristConstants.WRIST_KV);

  private final SparkMaxPIDController wristController = wristMotor.getPIDController();

  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  private final AnalogPotentiometer wristPotentiometer = new AnalogPotentiometer(WristConstants.POT_CHANEL,
      WristConstants.WRIST_POT_SCALE, WristConstants.WRIST_POT_OFFSET);

  // TODO: private final DigitalInput isStowed = new
  // DigitalInput(IntakeConstants.WRIST_LIMIT_SWITCH_CHANEL);

  private Timer time = new Timer();

  private double dt, lastTime;
  private double setpoint = WristPosition.STOW.angle();

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(WristConstants.CURRENT_LIMIT);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristEncoder.setPositionConversionFactor(WristConstants.WRIST_ENCODER_CONVERSION_FACTOR);
    wristEncoder.setVelocityConversionFactor(WristConstants.WRIST_ENCODER_CONVERSION_FACTOR / 60);
    wristEncoder.setPosition(WristConstants.MAX_WRIST_ANGLE);

    wristController.setP(WristConstants.WRIST_KP);
    wristController.setI(WristConstants.WRIST_KI);
    wristController.setD(WristConstants.WRIST_KD);
    wristController.setOutputRange(WristConstants.MAX_DOWNWARD_WRIST_SPEED, WristConstants.MAX_UPWARD_WRIST_SPEED);
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

    //System.out.println("wrist at setpoint"+atSetpoint());
  
  }

  public void turnOnBrakes(Boolean isBraked)
  {
    if (isBraked)
    {
        wristMotor.setIdleMode(IdleMode.kBrake);
    }
    else
    {
      wristMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  

  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  public double getWristVelocity() {
    return wristEncoder.getVelocity();
  }

  public void setPosition(double position, Supplier<Double> armAngleSupplier) {
      setpoint = Math.min(position, WristConstants.MAX_WRIST_ANGLE);
      setpoint = Math.max(setpoint, WristConstants.MIN_WRIST_ANGLE);
      wristController.setReference(position, CANSparkMax.ControlType.kPosition, 0,
          wristFeedforward.calculate(Units.degreesToRadians(setpoint), 0));
  }

  public void setPosition(WristPosition position, Supplier<Double> armAngleSupplier) {
    setPosition(position.angle(), armAngleSupplier);
  }

  public void setVelocity(double velocityDegrees, Supplier<Double> armAngleSupplier) {
      setpoint = setpoint + velocityDegrees * dt;
      setpoint = Math.min(setpoint, WristConstants.MAX_WRIST_ANGLE);
      setpoint = Math.max(setpoint, WristConstants.MIN_WRIST_ANGLE);
      wristController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
          wristFeedforward.calculate(Units.degreesToRadians(setpoint), Units.degreesToRadians(velocityDegrees)));
  }

  public double getWristMotorTemp() {
    return wristMotor.getMotorTemperature();
  }

  // TODO: Lukas. (High) Check this code is correct
  public boolean atSetpoint() {
    return (Math.abs(getWristPosition() - (setpoint)) < (WristConstants.WRIST_ANGLE_TOLERANCE));
  }

  public Command WaitUntilWristInPosition() {
    return Commands.waitUntil(() -> atSetpoint());
  }

  public WristPosition evalPickupLocation(Supplier<PickupLocation> location, Supplier<PieceType> piece) {
    if (location.get().equals(PickupLocation.FLOOR) && piece.get().equals(PieceType.CONE))
      return WristPosition.FLOOR_CONE_PICKUP;
    else if (location.get().equals(PickupLocation.FLOOR) && piece.get().equals(PieceType.CUBE))
      return WristPosition.FLOOR_CUBE_PICKUP;
    else if (location.get().equals(PickupLocation.DOUBLE))
      return WristPosition.DOUBLE_PICKUP;
    else
      return WristPosition.STOW;
  }

  public WristPosition evalScorePosition(Supplier<ScoreLevel> level) {
    if (level.get().equals(ScoreLevel.LEVEL_1))
      return WristPosition.LEVEL_1_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_2))
      return WristPosition.LEVEL_2_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_3))
      return WristPosition.LEVEL_2_CONE;
    else
      return WristPosition.LEVEL_2_CONE;
  }
}
