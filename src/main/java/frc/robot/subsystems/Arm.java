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
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;

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

  public Command moveToPosition(ArmConstants.Positions position) {
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

  public void setShoulderSpeed(double speed) {
    if (shoulderAngle.get() < ArmConstants.UPPER_ANGLE_LIMIT && speed > 0) {
      shoulderMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_SPEED, ArmConstants.MAX_SPEED));
    } else if (!isArmStowed.get() && speed < 0) {
      shoulderMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_SPEED, ArmConstants.MAX_SPEED));
    } else {
      shoulderMotor.set(0);
    }
  }

  public void actuateSuperstructure(PnuematicPositions position) {
    if (position == PnuematicPositions.EXTENDED) {
      leftPiston.set(Value.kForward);
      rightPiston.set(Value.kForward);
    } else if (position == PnuematicPositions.RETRACTED) {
      leftPiston.set(Value.kReverse);
      rightPiston.set(Value.kReverse);
    }
  }

  public Value superstructureState(){
    return leftPiston.get();
  }
}
