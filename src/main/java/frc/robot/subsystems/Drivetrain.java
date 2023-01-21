// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_WHEEL_PORT,MotorType.kBrushless);
  private final DifferentialDrive drivetrain;
  private String mode;
  public void setDriveTrainMode (String mode){
    this.mode = mode;
  }
  public Drivetrain() {
    drivetrain = new DifferentialDrive(leftFrontMotor,rightFrontMotor);
    drivetrain.setMaxOutput(.25);
    mode = Constants.TANK_DRIVE_STRING;
  }

  @Override
  public void periodic() {
    if (mode.equals(Constants.ARCADE_DRIVE_STRING)){
      arcadedrive(0, 0);
    }
    else if (mode.equals(Constants.TANK_DRIVE_STRING)){
      tankdrive(0, 0);
    }
  } 
   public void drive(double leftY, double rightX, double rightY ){
    }
  private void tankdrive(double leftSpeed, double rightSpeed){
      drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
  private void arcadedrive(double xSpeed, double zRotation){
    drivetrain.arcadeDrive(xSpeed, zRotation);
}

}
