/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Drive extends SubsystemBase {

  // Declare drive motor controllers
  private CANSparkMax leftFront, leftRear, rightFront, rightRear;

  /**
   * Creates a new Drive.
   */
  public Drive() {

    // Initialize motor controllers using ID's stored in Constants
    leftFront = new CANSparkMax(Constants.ID_LEFT_FRONT, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.ID_RIGHT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.ID_LEFT_REAR, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.ID_RIGHT_REAR, MotorType.kBrushless);

    // Set right front controller to be inverted
    leftFront.setInverted(false);
    rightFront.setInverted(true);

    // Set rear controllers to follow front
    leftRear.follow(leftFront, false);
    rightRear.follow(rightFront, false);

  }

  // Methods to set speed of left and right motor controllers
  public void setLeftSpeed(double speed) {
    leftFront.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightFront.set(speed);
  }

  // Set left and right drive speeds to zero
  public void stop() {
    setLeftSpeed(0);
    setRightSpeed(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
