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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.armIntakeCordinatorUtil;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;
import frc.robot.armIntakeCordinatorUtil.PickupPlaces;
import frc.robot.armIntakeCordinatorUtil.PieceType;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax shoulderMotor = new CANSparkMax(ArmConstants.SHOULDER_MOTOR_PORT, MotorType.kBrushless);
  DoubleSolenoid leftPiston;
  DoubleSolenoid rightPiston;

  TrapezoidProfile.Constraints shoulderConstraints = new Constraints(ArmConstants.MAX_SHOULDER_VELOCITY,
      ArmConstants.MAX_SHOULDER_ACCELERATION);
  TrapezoidProfile shoulderProfile = new TrapezoidProfile(shoulderConstraints, null, null);

  AnalogPotentiometer shoulderAngle = new AnalogPotentiometer(ArmConstants.SHOULDER_POTENTIOMETER_PORT, 270, -135);
  DigitalInput isArmStowed = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

  public Arm(Pnuematics pnuematics) {
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    leftPiston = pnuematics.getLeftArmPiston();
    rightPiston = pnuematics.getRightArmPiston();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command moveArmPickupCommand(armIntakeCordinatorUtil cordinatorUtil, PickupPlaces location) {
    SequentialCommandGroup command = null;
    if (location.equals(PickupPlaces.FLOOR)) {
      command = new SequentialCommandGroup(moveToPositionCommand(ArmPositions.FLOOR_PICKUP),
          actuateSuperstructureCommand(ArmPositions.FLOOR_PICKUP));
    }

    else if (location.equals(PickupPlaces.DOUBLE)) {
      command = new SequentialCommandGroup(moveToPositionCommand(ArmPositions.STATION_PICKUP),
          actuateSuperstructureCommand(ArmPositions.STATION_PICKUP));
    }
    return command;
  }

  public Command moveArmScoreCommand(armIntakeCordinatorUtil cordinatorUtil) {
    SequentialCommandGroup command = new SequentialCommandGroup(
        moveToPositionCommand(cordinatorUtil.getScoreArmPosition()),
        actuateSuperstructureCommand(cordinatorUtil.getScoreArmPosition()));
    return command;
  }

  public Command moveToPositionCommand(ArmConstants.ArmPositions position) {
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
    ProfiledPIDCommand command = new ProfiledPIDCommand(
        new ProfiledPIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, shoulderConstraints),
        this.shoulderAngle::get,
        new TrapezoidProfile.State(position.getShoulderAngle(), 0),
        (output, setpoint) -> {
          this.shoulderMotor.set(output + feedforward.calculate(setpoint.position, setpoint.velocity));
        },
        this);

    command.getController().setTolerance(ArmConstants.POSITION_TOLERANCE);
    return command;

  }

  public void moveToPosition(){

  }

  public void setShoulderSpeed(double speed) {
    if (shoulderAngle.get() < ArmConstants.UPPER_ANGLE_LIMIT && speed > 0) {
      shoulderMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_SPEED, ArmConstants.MAX_SPEED));
    } else if (!isArmStowed.get() && speed < 0) {
      shoulderMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_SPEED, ArmConstants.MAX_SPEED));
    } else {
      shoulderMotor.set(0);
    }
  }

  public Command actuateSuperstructureCommand(ArmPositions position) {
    return new InstantCommand(() -> actuateSuperstructure(position.getPistonPosition()), this);
  }

  public Command actuateSuperstructureCommand(PnuematicPositions position) {
    return new InstantCommand(() -> actuateSuperstructure(position), this);
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
