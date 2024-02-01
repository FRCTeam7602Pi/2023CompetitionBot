// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR_CONTROLLER;
import static frc.robot.Constants.Elevator.MAX_POSITION;
import static frc.robot.Constants.Elevator.START_POSITION;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PidElevator extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;

  // free form string used to communicate state to dashboard
  private String command = "";

  // PID config
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public PidElevator() {
    m_motor = new CANSparkMax(ELEVATOR_CONTROLLER, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    // PID config - test and tweak here
    kP = 0.25;
    kI = 0;
    kD = 1;
    kIz = 0;
    kFF = 1e-4;
    kMaxOutput = .4;
    kMinOutput = -.25;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // allow config to be viewed and adjusted via SmartDashboard
    SmartDashboard.putNumber("Elevator P Gain", kP);
    SmartDashboard.putNumber("Elevator I Gain", kI);
    SmartDashboard.putNumber("Elevator D Gain", kD);
    SmartDashboard.putNumber("Elevator I Zone", kIz);
    SmartDashboard.putNumber("Elevator Feed Forward", kFF);
    SmartDashboard.putNumber("Elevator Max Output", kMaxOutput);
    SmartDashboard.putNumber("Elevator Min Output", kMinOutput);
    SmartDashboard.putNumber("Elevator Set Rotations", 0);
  }

  @Override
  public void periodic() {
    updateDashboard();
  }

  public void elevatorTop() {
    command = "TOP";
    setPosition(MAX_POSITION);
  }

  public void elevatorBottom() {
    command = "BOTTOM";
    setPosition(START_POSITION);
  }

  public void moveUp() {
    command = "UP";
    setPosition(m_encoder.getPosition() + 1);
  }

  public void moveDown() {
    command = "DOWN";
    setPosition(m_encoder.getPosition() - 1);
  }

  public void stop() {
    command = "STOP";
  }

  public boolean isBottom() {
    return (m_encoder.getPosition() - START_POSITION) <= 1.0f;
  }

  public boolean isTop() {
    return (MAX_POSITION - m_encoder.getPosition()) <= 1.0f;
  }

  private void ignoreCommand() {
    command = command + " (ignored)";
  }

  private void setPosition(double position) {
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  private void updateDashboard() {
    double position = m_encoder.getPosition();
    SmartDashboard.putNumber("Elevator Encoder Position", position);
    SmartDashboard.putString("Elevator Last Command", command);

    // read PID config in case user wants to update
    double p = SmartDashboard.getNumber("Elevator P Gain", 0);
    double i = SmartDashboard.getNumber("Elevator I Gain", 0);
    double d = SmartDashboard.getNumber("Elevator D Gain", 0);
    double iz = SmartDashboard.getNumber("Elevator I Zone", 0);
    double ff = SmartDashboard.getNumber("Elevator Feed Forward", 0);
    double max = SmartDashboard.getNumber("Elevator Max Output", 0);
    double min = SmartDashboard.getNumber("Elevator Min Output", 0);
    double rotations = SmartDashboard.getNumber("Elevator Set Rotations", 0);

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

    SmartDashboard.putNumber("Elevator SetPoint", rotations);
    SmartDashboard.putNumber("Elevator ProcessVariable", m_encoder.getPosition());
  }
}
