// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_WHEEL_PORT,MotorType.kBrushless);
  private final DifferentialDrive drivetrain;
  private final SendableChooser<String> drivetrainSelector = new SendableChooser<>();
  private String drivemode;
  public Drivetrain() {
    drivetrain = new DifferentialDrive(leftFrontMotor,rightFrontMotor);
    drivetrain.setMaxOutput(.25);
    drivetrainSelector.setDefaultOption("Tank", Constants.TANKMODE);
    drivetrainSelector.addOption("Arcade", Constants.ARCADEMODE);
    SmartDashboard.putData("Drivetrain Mode", drivetrainSelector);
  }

  @Override
  public void periodic() {
    drivemode = drivetrainSelector.getSelected();
  } 
   public void drive(double leftY, double rightX, double rightY ){
    switch (drivemode) {
      case Constants.TANKMODE:
        tankdrive(leftY, rightY);
        break;
      case Constants.ARCADEMODE:
        arcadedrive(leftY, rightX);
        break;
      default:
        tankdrive(leftY, rightY);
        break;
    }
   }
  private void tankdrive(double leftSpeed, double rightSpeed){
      drivetrain.tankDrive(leftSpeed, rightSpeed);
  }
  private void arcadedrive(double xSpeed, double zRotation){
    drivetrain.arcadeDrive(xSpeed, zRotation);
}

}
