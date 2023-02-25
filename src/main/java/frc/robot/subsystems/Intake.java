// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.IntakeConstants.IntakeDirection;
import frc.robot.Constants.IntakeConstants.WristPosition;


public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax wristMotor = new CANSparkMax(IntakeConstants.WRIST_CAN_ID, MotorType.kBrushless);
  private final AnalogPotentiometer wristPotentiometer = new AnalogPotentiometer(IntakeConstants.POT_CHANEL,
      IntakeConstants.WRIST_POT_SCALE, IntakeConstants.WRIST_POT_OFFSET);
  private final DigitalInput isHoldingCube = new DigitalInput(IntakeConstants.DISTANCE_SENSOR_CHANEL);
  private final DigitalInput isIntakeStored = new DigitalInput(IntakeConstants.INTAKE_LIMIT_SWITCH_ID);

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

  public double getWristAngle() {
    return wristPotentiometer.get();
  }

  public boolean isHoldingGamePiece() {
    // add in code for holding a cone
    return isHoldingCube.get();

  }

  public void setWristSpeed(double speed) {
    //TODO:
    if ((speed < 0.0)  /*&&(isStored() == false)*/) {
      wristMotor.set(MathUtil.clamp(speed, -IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_SPEED));
    } else if ((speed > 0.0) && (wristPotentiometer.get() <= IntakeConstants.MAX_WRIST_ANGLE)) {
      wristMotor.set(MathUtil.clamp(speed, -IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_SPEED));
    } else {
      wristMotor.set(0.0);
    }
  }

  public boolean isStored() {
    return isIntakeStored.get();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }



  public Command SpinIntakeCommand(Supplier<PieceType> piece, boolean isIntaking) {

    return Commands.startEnd((() -> this.setIntakeSpeed(evalPieceIntake(piece.get(), isIntaking).speed())), (() -> this.setIntakeSpeed(IntakeDirection.STOP.speed())) , this);
  }

  public Command moveToPositionCommand(PickupLocation location, Supplier<PieceType> piece) {
    ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG,
        IntakeConstants.WRIST_KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(IntakeConstants.WRIST_KP,
            IntakeConstants.WRIST_KI, IntakeConstants.WRIST_KD,
            new TrapezoidProfile.Constraints(IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_ACCEL)),
        () -> this.getWristAngle(),
        () -> evalPickupLocation(location, piece.get()).angle(),
        (output, setpoint) -> {
          this.setWristSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(IntakeConstants.WRIST_ANGLE_TOLERANCE);

    return command;
  }

  public Command holdCommand(){
    ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG, IntakeConstants.WRIST_KV);
    return Commands.runEnd(() -> setWristSpeed(feedforward.calculate(getWristAngle(), 0)),
        () -> setWristSpeed(0), this);
  }

  public Command moveToPositionCommand(ScoreLevel level) {
    ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG,
        IntakeConstants.WRIST_KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(IntakeConstants.WRIST_KP,
            IntakeConstants.WRIST_KI, IntakeConstants.WRIST_KD,
            new TrapezoidProfile.Constraints(IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_ACCEL)),
        () -> this.getWristAngle(),
        () -> evalScorePosition(level).angle(),
        (output, setpoint) -> {
          this.setWristSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(IntakeConstants.WRIST_ANGLE_TOLERANCE);

    return command;
  }

  public Command stowCommand(){
    ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG,
        IntakeConstants.WRIST_KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(IntakeConstants.WRIST_KP,
            IntakeConstants.WRIST_KI, IntakeConstants.WRIST_KD,
            new TrapezoidProfile.Constraints(IntakeConstants.MAX_WRIST_SPEED, IntakeConstants.MAX_WRIST_ACCEL)),
        () -> this.getWristAngle(),
        () -> WristPosition.STOW.angle(),
        (output, setpoint) -> {
          this.setWristSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(IntakeConstants.WRIST_ANGLE_TOLERANCE);

    return command;
  }

  private WristPosition evalPickupLocation(PickupLocation location, PieceType piece) {
    if (location.equals(PickupLocation.FLOOR) && piece.equals(PieceType.CONE))
      return WristPosition.FLOOR_CONE_PICKUP;
    else if (location.equals(PickupLocation.FLOOR) && piece.equals(PieceType.CUBE))
      return WristPosition.FLOOR_CUBE_PICKUP;
    else if (location.equals(PickupLocation.DOUBLE))
      return WristPosition.DOUBLE_PICKUP;
    else
      return WristPosition.STOW;
  }

  private WristPosition evalScorePosition(ScoreLevel level) {
    if (level.equals(ScoreLevel.LEVEL_1))
      return WristPosition.LEVEL_1;
    else if (level.equals(ScoreLevel.LEVEL_2))
      return WristPosition.LEVEL_2;
    else if (level.equals(ScoreLevel.LEVEL_3))
      return WristPosition.LEVEL_2;
    else
      return WristPosition.LEVEL_2;
  }

  private IntakeDirection evalPieceIntake(PieceType piece, boolean isIntaking){
    if(piece.equals(PieceType.CONE) && isIntaking)
      return IntakeDirection.PICK_CONE;
    else if(piece.equals(PieceType.CUBE) && isIntaking)
      return IntakeDirection.PICK_CUBE;
    if(piece.equals(PieceType.CONE) && !isIntaking)
      return IntakeDirection.PLACE_CONE;
    else if(piece.equals(PieceType.CUBE) && !isIntaking)
      return IntakeDirection.PLACE_CUBE;
    else   
      return IntakeDirection.STOP;
  }

  public Command scoreGamePieceCommand(IntakeConstants.WristPosition hightPosition,
      IntakeConstants.IntakeDirection direction) {
    return null;
  }

  public double getWristSpeed() {
    return wristMotor.getEncoder().getVelocity();
  }

  public double getIntakeSpeed() {
    return intakeMotor.getEncoder().getVelocity();
  }

}
