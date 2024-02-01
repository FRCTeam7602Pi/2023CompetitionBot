// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;

  // free form string used to communicate state to dashboard
  private String command = "";

  public Arm() {
    armMotor = new CANSparkMax(Constants.ARM_CONTROLLER, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setInverted(true);
    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    updateDashboard();
  }

  /**
   * Extend the arm by driving motor at given rate.  A positive number would
   * push the arm out and a negative number brings it back in.
   * 
   * We expect the arm to hold position when not getting any other input.
   */
  public void extend(double rate) {
    command = "EXTEND";
    if (!isFullyExtended()) {
      armMotor.set(rate * Constants.ARM_MOVE_VELOCITY);
    } else {
      ignoreCommand();
    }
  }

  public void retract(double rate) {
    command = "RETRACT";
    if (!isFullyRetracted()) {
      armMotor.set(-rate * Constants.ARM_MOVE_VELOCITY);
    } else {
      ignoreCommand();
    }
  }

  public void stop() {
    command = "STOP";
    armMotor.stopMotor();
  }

  public boolean isFullyRetracted() {
    return (armEncoder.getPosition() - Constants.Arm.START_POSITION) <= 1.0f;
  }

  public boolean isFullyExtended() {
    return (Constants.Arm.MAX_POSITION - armEncoder.getPosition()) <= 1.0f;
  }

  private void ignoreCommand() {
    command = command + " (ignored)";
  }

  private void updateDashboard() {
    double position = armEncoder.getPosition();
    SmartDashboard.putNumber("Arm Encoder Position", position);
    SmartDashboard.putString("Arm Last Command", command);
  }
}
