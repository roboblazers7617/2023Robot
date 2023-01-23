// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.LEFT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.RIGHT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax leftFollowerMotor = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFollowerMotor = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_WHEEL_PORT,MotorType.kBrushless);
  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftFollowerMotor);
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightFollowerMotor);
  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder leftFollowerEncoder = leftFollowerMotor.getEncoder();
  private final RelativeEncoder rightFollowerEncoder = rightFollowerMotor.getEncoder();

  private final DifferentialDrive drivetrain;
  private String mode;
  private double maxDrivetrainspeed = DrivetrainConstants.MAX_SPEED;
  public void setDriveTrainMode (String mode){
    this.mode = mode;
  }
  public Drivetrain() {
    drivetrain = new DifferentialDrive(leftMotorGroup,rightMotorGroup);
    drivetrain.setMaxOutput(DrivetrainConstants.MAX_SPEED);
    mode = DrivetrainConstants.TANK_DRIVE_STRING;
    configureEncoder(leftFrontEncoder);
    configureEncoder(rightFrontEncoder);
    configureEncoder(leftFollowerEncoder);
    configureEncoder(rightFollowerEncoder);
    
    configureMotor(leftFrontMotor);
    configureMotor(rightFrontMotor);
    configureMotor(leftFollowerMotor);
    configureMotor(rightFollowerMotor);

    rightMotorGroup.setInverted(true);
    drivetrain.setDeadband(.1);


  }

  @Override
  public void periodic() {
  } 
   public void drive(double leftY, double rightX, double rightY ){
    if (mode.equals(DrivetrainConstants.ARCADE_DRIVE_STRING)){
      arcadeDrive(-leftY, -rightX);
    }
    else if (mode.equals(DrivetrainConstants.TANK_DRIVE_STRING)){
      tankDrive(-leftY, -rightY);
    }


    }
  private void tankDrive(double leftSpeed, double rightSpeed){
      drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
  
  public void arcadeDrive(double xSpeed, double zRotation){
    drivetrain.arcadeDrive(xSpeed, zRotation);
}

public void setDrivetrainSpeed(double maxSpeed){
  maxDrivetrainspeed = maxSpeed;
  drivetrain.setMaxOutput(maxSpeed);
}

public double getCarmax(){
  return maxDrivetrainspeed;
}

private void configureMotor (CANSparkMax motorController){
  motorController.restoreFactoryDefaults();
  motorController.setIdleMode(IdleMode.kCoast);
  motorController.setSmartCurrentLimit(DrivetrainConstants.CURRENT_LIMIT);
}

private void configureEncoder(RelativeEncoder motorEncoder){
  motorEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION);
  motorEncoder.setPosition(0);
}
}
