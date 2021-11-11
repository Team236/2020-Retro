/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Drive extends SubsystemBase {

  private CANSparkMax leftFront, leftRear, rightFront, rightRear;
  private CANEncoder leftEncoder, rightEncoder;
  private AHRS navx;
  private CANPIDController leftPID, rightPID;

  /**
   * Creates a new Drive.
   */
  public Drive() {

    leftFront = new CANSparkMax(Constants.ID_LEFT_FRONT, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.ID_RIGHT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.ID_LEFT_REAR, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.ID_RIGHT_REAR, MotorType.kBrushless);

    navx = new AHRS(Port.kMXP);

    leftFront.setInverted(false);
    rightFront.setInverted(true);

    leftRear.follow(leftFront, false);
    rightRear.follow(rightFront, false);

    leftPID = leftFront.getPIDController();
    leftPID = leftFront.getPIDController();

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

  }

  public void setLeftSpeed(double speed) {
    leftFront.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightFront.set(speed);
  }
  // sets speed to zero if between -0.1 to 0.1
  public void setLeftSpeedWithDeadzone(double speed) {
    double leftSpeed = speed;
    if (leftSpeed < Constants.LEFT_DEADZONE && leftSpeed > -Constants.LEFT_DEADZONE) {
      leftSpeed = 0;
    }

    setLeftSpeed(leftSpeed);
  }

  public void setRightSpeedWithDeadzone(double speed) {
    double rightSpeed = speed;
    if (rightSpeed < Constants.LEFT_DEADZONE && rightSpeed > -Constants.RIGHT_DEADZONE) {
      rightSpeed = 0;
    }

    setRightSpeed(rightSpeed);
  }

  public double getGyroAngle() {
    return navx.getAngle();
    // return 0;
  }

  // @Override
  public void resetAngle() {
    navx.reset();
  }

  // @Override
  public double getRate() {
    return navx.getRate();
    // return 0;
  }

  // sets both left/right drive speeds to zero
  public void stop() {
    setLeftSpeed(0);
    setRightSpeed(0);
  }

  //gets value of Neo encoders
  public double getLeftEncoder() {
    return leftEncoder.getPosition() * Constants.REV_TO_IN_K;
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition() * Constants.REV_TO_IN_K;
  }

  public double getRightDistance() {
    return getRightEncoder() * Constants.REV_TO_IN_K;
  }

  public double getLeftDistance() {
    return getLeftEncoder() * Constants.REV_TO_IN_K;
  }

  // resets Neo drive motor encoders
  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }
  
  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  // PID 

  // @Override
  // public double pidGet() {
  //   return getGyroAngle();
  // }

  // @Override
  // public void pidSet(double speed) {
  //   setRightSpeed(-speed);
  //   setLeftSpeed(speed);
  // }

  // SPARK Motion Control kP

  // public void setkP(double kP) {
  //   leftPID.setP(kP);
  //   rightPID.setP(kP);
  // }

  // public void setkI(double kI) {
  //   leftPID.setI(kI);
  //   rightPID.setI(kI);
  // }

  // public void setkD(double kD) {
  //   leftPID.setD(kD);
  //   rightPID.setD(kD);
  // }

  // public void setkF(double kF) {
  //   leftPID.setFF(kF);
  //   rightPID.setFF(kF);
  // }

  // public void setOutputRange(double minOutput, double maxOutput) {
  //   leftPID.setOutputRange(minOutput, maxOutput);
  //   rightPID.setOutputRange(minOutput, maxOutput);
  // }

  // public void setSetPoint(double dist) {
  //   leftPID.setReference((dist * Constants.IN_TO_REV_K), ControlType.kPosition);
  //   rightPID.setReference((dist * Constants.IN_TO_REV_K), ControlType.kPosition);
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Drive Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right Drive Encoder", getRightEncoder());

    SmartDashboard.putNumber("navx angle", getGyroAngle());
  }
}
