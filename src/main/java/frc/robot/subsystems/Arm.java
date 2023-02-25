// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.ScoreLevel;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax shoulderMotor = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
  DoubleSolenoid leftPiston;
  DoubleSolenoid rightPiston;

  TrapezoidProfile.Constraints shoulderConstraints = new Constraints(ArmConstants.MAX_SHOULDER_VELOCITY,
      ArmConstants.MAX_SHOULDER_ACCELERATION);

  AnalogPotentiometer shoulderAngle = new AnalogPotentiometer(ArmConstants.SHOULDER_POTENTIOMETER_PORT, ArmConstants.SHOULDER_POTENTIOMETER_RANGE, ArmConstants.SHOULDER_POTENTIOMETER_OFFSET);
  DigitalInput isArmStowed = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

  public Arm(Pnuematics pnuematics) {
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kCoast);
    shoulderMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    leftPiston = pnuematics.getLeftArmPiston();
    rightPiston = pnuematics.getRightArmPiston();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command moveToPositionCommand(ScoreLevel level) {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, shoulderConstraints),
        this.shoulderAngle::get,
        new TrapezoidProfile.State(evalScorePosition(level).getShoulderAngle(), 0),
        (output, setpoint) -> {
          this.setShoulderSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(ArmConstants.POSITION_TOLERANCE);
    return command;

  }

  public Command moveToPositionCommand(PickupLocation location) {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, shoulderConstraints),
        this.shoulderAngle::get,
        new TrapezoidProfile.State(evalPickupPosition(location).getShoulderAngle(), 0),
        (output, setpoint) -> {
          this.setShoulderSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(ArmConstants.POSITION_TOLERANCE);
    return command;

  }

  public Command stowCommand() {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, shoulderConstraints),
        this.shoulderAngle::get,
        new TrapezoidProfile.State(ArmPositions.STOW.getShoulderAngle(), 0),
        (output, setpoint) -> {
          this.setShoulderSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(ArmConstants.POSITION_TOLERANCE);

    return new SequentialCommandGroup(actuateSuperstructureCommand(ArmPositions.STOW.getPistonPosition()), command);

  }

  
  public Command moveToHeldPositionCommand(PickupLocation location) {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, shoulderConstraints),
        this.shoulderAngle::get,
        new TrapezoidProfile.State(evalPickupPosition(location).getShoulderAngle(), 0),
        (output, setpoint) -> {
          this.setShoulderSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);
    return new SequentialCommandGroup(command, Commands.runOnce(() -> {
      setShoulderSpeed(feedforward.calculate(getShoulderAngle(), 0));
      actuateSuperstructure(evalPickupPosition(location).getPistonPosition());
    },
     this));
  }

  public Command moveToHeldPositionCommand(ScoreLevel level) {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, shoulderConstraints),
        this.shoulderAngle::get,
        new TrapezoidProfile.State(evalScorePosition(level).getShoulderAngle(), 0),
        (output, setpoint) -> {
          this.setShoulderSpeed(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);
    return new SequentialCommandGroup(command, Commands.runOnce(() -> {
      setShoulderSpeed(feedforward.calculate(getShoulderAngle(), 0));
      actuateSuperstructure(evalScorePosition(level ).getPistonPosition());
    },
     this));
  }

  public Command holdCommand() {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    return Commands.runEnd(() -> setShoulderSpeed(feedforward.calculate(getShoulderAngle(), 0)),
        () -> setShoulderSpeed(0), this);
  }

  private ArmPositions evalPickupPosition(PickupLocation location) {
    if (location.equals(PickupLocation.FLOOR))
      return ArmPositions.FLOOR_PICKUP;
    else if (location.equals(PickupLocation.DOUBLE))
      return ArmPositions.STATION_PICKUP;
    else
      return ArmPositions.STOW;
  }

  private ArmPositions evalScorePosition(ScoreLevel level) {
    if (level.equals(ScoreLevel.LEVEL_1))
      return ArmPositions.LEVEL_1;
    else if (level.equals(ScoreLevel.LEVEL_2))
      return ArmPositions.LEVEL_2;
    else if (level.equals(ScoreLevel.LEVEL_3))
      return ArmPositions.LEVEL_3;
    else
      return ArmPositions.STOW;
  }

  public void setShoulderSpeed(double speed) {
    if (shoulderAngle.get() < ArmConstants.UPPER_ANGLE_LIMIT && speed > 0) {
      shoulderMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_SPEED, ArmConstants.MAX_SPEED));

      // TODO: Add limit switch
    } else if (shoulderAngle.get() > 80/*!isArmStowed.get() && speed*/&& speed < 0) {
      shoulderMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_SPEED, ArmConstants.MAX_SPEED));
    } else {
      shoulderMotor.set(0);
    }
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

  public double getShoulderAngle() {
    return shoulderAngle.get();
  }

  public boolean isArmStowed() {
    return isArmStowed.get();
  }
}
