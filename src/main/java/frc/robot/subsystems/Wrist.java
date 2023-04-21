// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

  // TODO: private final DigitalInput isStowed = new
  // DigitalInput(IntakeConstants.WRIST_LIMIT_SWITCH_CHANEL);

  private Timer time = new Timer();

  private double dt, lastTime;
  private double setpoint = WristPosition.STOW.angle();

  private double maxAngle = WristConstants.MAX_WRIST_ANGLE;

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(WristConstants.CURRENT_LIMIT);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristEncoder.setPositionConversionFactor(WristConstants.WRIST_ENCODER_CONVERSION_FACTOR);
    wristEncoder.setVelocityConversionFactor(WristConstants.WRIST_ENCODER_CONVERSION_FACTOR);
    wristEncoder.setPosition(102);


    wristController.setP(WristConstants.WRIST_KP);
    wristController.setI(WristConstants.WRIST_KI);
    wristController.setD(WristConstants.WRIST_KD);
    wristController.setFeedbackDevice(wristEncoder);
    wristController.setOutputRange(WristConstants.MAX_DOWNWARD_WRIST_SPEED, WristConstants.MAX_UPWARD_WRIST_SPEED);
    //wristController.setSmartMotionMaxAccel(WristConstants.MAX_ACCEL, 0);
   // wristController.setSmartMotionMaxVelocity(WristConstants.MAX_VEL, 0);
   //TODO: fix this
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

    // System.out.println("wrst at setpoint"+atSetpoint());

  }

  public void turnOnBrakes(Boolean isBraked) {
    if (isBraked) {
      wristMotor.setIdleMode(IdleMode.kBrake);
    } else {
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
    setpoint = Math.min(position, maxAngle);
    wristController.setReference(position, CANSparkMax.ControlType.kPosition, 0,
        wristFeedforward.calculate(
            Units.degreesToRadians(setpoint)+ (armAngleSupplier.get() - ArmConstants.FF_MAX_SHOULDER_ANGLE), 0));
  }

  public void setPosition(WristPosition position, Supplier<Double> armAngleSupplier) {
    setPosition(position.angle(), armAngleSupplier);
  }

  public void setVelocity(double velocityDegrees, Supplier<Double> armAngleSupplier) {
    setpoint = setpoint + velocityDegrees * dt;
    setpoint = Math.min(setpoint, maxAngle);
    wristController.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0,
        wristFeedforward.calculate(
            Units.degreesToRadians(setpoint)+ (armAngleSupplier.get() - ArmConstants.FF_MAX_SHOULDER_ANGLE),
            Units.degreesToRadians(velocityDegrees)));
  }

  public double getWristMotorTemp() {
    return wristMotor.getMotorTemperature();
  }
  
  //public void resetEncoder(){
  //  wristEncoder.setPosition(WristConstants.MAX_WRIST_ANGLE);
  //  setPosition(WristConstants.MAX_WRIST_ANGLE, () -> ArmConstants.MINIMUM_SHOULDER_ANGLE);
 // }

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
    else if (location.get().equals(PickupLocation.DOUBLE) && piece.get().equals(PieceType.CONE))
      return WristPosition.DOUBLE_PICKUP_CONE;
    else if (location.get().equals(PickupLocation.DOUBLE) && piece.get().equals(PieceType.CUBE))
      return WristPosition.DOUBLE_PICKUP_CUBE;
    else
      return WristPosition.STOW;
  }

  public WristPosition evalScorePosition(Supplier<ScoreLevel> level, Supplier<PieceType> piece) {
    if (level.get().equals(ScoreLevel.LEVEL_1 )&& piece.get().equals(PieceType.CONE))
      return WristPosition.LEVEL_1_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_2) && piece.get().equals(PieceType.CONE))
      return WristPosition.LEVEL_2_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_3) && piece.get().equals(PieceType.CONE))
      return WristPosition.LEVEL_3_CONE;
    else if (level.get().equals(ScoreLevel.LEVEL_1 )&& piece.get().equals(PieceType.CUBE))
      return WristPosition.LEVEL_1_CUBE;
    else if (level.get().equals(ScoreLevel.LEVEL_2) && piece.get().equals(PieceType.CUBE))
      return WristPosition.LEVEL_2_CUBE;
    else if (level.get().equals(ScoreLevel.LEVEL_3) && piece.get().equals(PieceType.CUBE))
      return WristPosition.LEVEL_3_CUBE;
    else
      return WristPosition.STOW;
  }

public void removeBounds() {
	 maxAngle = 200;
}

public void addBounds() {
     maxAngle = WristConstants.MAX_WRIST_ANGLE;
}

public void resetEncoder() {
    wristEncoder.setPosition(102);
}
}
