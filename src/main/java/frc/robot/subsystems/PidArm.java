// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ARM_CONTROLLER;
import static frc.robot.Constants.ARM_MOVE_VELOCITY;
import static frc.robot.Constants.Arm.MAX_POSITION;
import static frc.robot.Constants.Arm.START_POSITION;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PidArm extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;

  // free form string used to communicate state to dashboard
  private String command = "";

  // PID config
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public PidArm() {
    m_motor = new CANSparkMax(ARM_CONTROLLER, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(true);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    // PID config for this subsystem
    kP = 0.1;
    kI = 0;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = .2;
    kMinOutput = -.2;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // allow config to be viewed and adjusted via SmartDashboard
    SmartDashboard.putNumber("Arm P Gain", kP);
    SmartDashboard.putNumber("Arm I Gain", kI);
    SmartDashboard.putNumber("Arm D Gain", kD);
    SmartDashboard.putNumber("Arm I Zone", kIz);
    SmartDashboard.putNumber("Arm Feed Forward", kFF);
    SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    SmartDashboard.putNumber("Arm Min Output", kMinOutput);
    SmartDashboard.putNumber("Arm Set Rotations", 0);
  }

  @Override
  public void periodic() {
    updateDashboard();

    // TODO - do we need to check if close to endpoints?
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
      // m_motor.set(rate * ARM_MOVE_VELOCITY);
      setPosition(m_encoder.getPosition() - 4);
    } else {
      setPosition(MAX_POSITION);
      warnLimitedCommand();
    }
  }

  public void retract(double rate) {
    command = "RETRACT";
    if (!isFullyRetracted()) {
      // m_motor.set(-rate * ARM_MOVE_VELOCITY);
      setPosition(m_encoder.getPosition() - 2);
    } else {
      setPosition(START_POSITION);
      warnLimitedCommand();
    }
  }

  public void stop() {
    command = "STOP";
    m_motor.stopMotor();

    // not sure yet if this is the correct method here but idea is to use
    // PID controller hold position rather than just motor shunt
    setPosition(m_encoder.getPosition());
  }

  public boolean isFullyRetracted() {
    // TODO - consider checking elevator height for case that retract below
    // bummer will be different than retracted above it
    return (m_encoder.getPosition() - START_POSITION) <= 1.0f;
  }

  public boolean isFullyExtended() {
    return (MAX_POSITION - m_encoder.getPosition()) <= 1.0f;
  }

  private void warnLimitedCommand() {
    command = command + " (ignored)";
  }

  private void setPosition(double position) {
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  private void updateDashboard() {
    double position = m_encoder.getPosition();
    SmartDashboard.putNumber("Arm Encoder Position", position);
    SmartDashboard.putString("Arm Last Command", command);

    // read PID config in case user wants to update
    double p = SmartDashboard.getNumber("Arm P Gain", 0);
    double i = SmartDashboard.getNumber("Arm I Gain", 0);
    double d = SmartDashboard.getNumber("Arm D Gain", 0);
    double iz = SmartDashboard.getNumber("Arm I Zone", 0);
    double ff = SmartDashboard.getNumber("Arm Feed Forward", 0);
    double max = SmartDashboard.getNumber("Arm Max Output", 0);
    double min = SmartDashboard.getNumber("Arm Min Output", 0);
    double rotations = SmartDashboard.getNumber("Arm Set Rotations", 0);

    // if (p != kP) {
    //   m_pidController.setP(p);
    //   kP = p;
    // }
    // if (i != kI) {
    //   m_pidController.setI(i);
    //   kI = i;
    // }
    // if (d != kD) {
    //   m_pidController.setD(d);
    //   kD = d;
    // }
    // if (iz != kIz) {
    //   m_pidController.setIZone(iz);
    //   kIz = iz;
    // }
    // if (ff != kFF) {
    //   m_pidController.setFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    //   m_pidController.setOutputRange(min, max);
    //   kMinOutput = min;
    //   kMaxOutput = max;
    // }

    // // TODO - replace this - for now, just controlling by SmartDashboard
    // m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("Arm SetPoint", rotations);
    SmartDashboard.putNumber("Arm ProcessVariable", m_encoder.getPosition());
  }
}
