// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Pincher.CONE_POSITION;
import static frc.robot.Constants.Pincher.CUBE_POSITION;
import static frc.robot.Constants.Pincher.MAX_POSITION;
import static frc.robot.Constants.Pincher.START_POSITION;
import static frc.robot.Constants.Pincher.OPEN_POSITION;
import static frc.robot.Constants.PINCHER_MOVE_VELOCITY;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PidPincher extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;

  // free form string used to communicate state to dashboard
  private String command = "";

  // PID config
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public PidPincher() {
    m_motor = new CANSparkMax(Constants.PINCHER_CONTROLLER, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(Constants.PINCHER_CURRENT_LIMIT);
    m_motor.setInverted(true);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    // PID config for this subsystem
    kP = 0.2;
    kI = 0;
    kD = 5.5;
    kIz = 0;
    kFF = 0.0015;
    kMaxOutput = .8;
    kMinOutput = -.8;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // allow config to be viewed and adjusted via SmartDashboard
    SmartDashboard.putNumber("Pincher P Gain", kP);
    SmartDashboard.putNumber("Pincher I Gain", kI);
    SmartDashboard.putNumber("Pincher D Gain", kD);
    SmartDashboard.putNumber("Pincher I Zone", kIz);
    SmartDashboard.putNumber("Pincher Feed Forward", kFF);
    SmartDashboard.putNumber("Pincher Max Output", kMaxOutput);
    SmartDashboard.putNumber("Pincher Min Output", kMinOutput);
    SmartDashboard.putNumber("Pincher Set Rotations", 0);
  }

  @Override
  public void periodic() {
    updateDashboard();

    // TODO - need to check endpoints to slow down or stop?
  }

  public void clampCube() {
    command = "Cube";
    setPosition(CUBE_POSITION);
  }

  public void clampStandingCone() {
    command = "Cone";
    setPosition(CONE_POSITION);
  }

  public void close(double velocity) {
    command = "Close";
    // TODO incorporate magnitude of axis
    if (m_encoder.getPosition()<MAX_POSITION) {
      setPosition(m_encoder.getPosition() + .4);
    }
  }

  public boolean isInCubePosition() {
    return (CUBE_POSITION - m_encoder.getPosition()) <= 1.0f;
  }

  public void open(double velocity) {
    command = "Open";
    // TODO incorporate magnitude of axis
    if (m_encoder.getPosition()>START_POSITION) {
      setPosition(m_encoder.getPosition() - .4);
    }
  }

  public void ready() {
    command = "Ready Position";
    setPosition(OPEN_POSITION);
  }

  public void stop() {
    System.out.println("Pincher Stop");
    m_motor.stopMotor();

    // not sure yet if this is the correct method but the idea is to use
    // PID controller to hold position rather than just motor shunt
    setPosition(m_encoder.getPosition());
  }

  private void setPosition(double position) {
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Apply scale from Constants to the given velocity.  This is primarily
   * for manual control since those are expected to be minor adjustments.
   */
  private double scale(double velocity) {
    return velocity / Constants.PINCHER_CONTROL_SCALING;
  }

  private void updateDashboard() {
    double position = m_encoder.getPosition();
    SmartDashboard.putNumber("Pincher Encoder Position", position);
    SmartDashboard.putString("Pincher Last Command", command);

    // read PID config in case user wants to update
    double p = SmartDashboard.getNumber("Pincher P Gain", 0);
    double i = SmartDashboard.getNumber("Pincher I Gain", 0);
    double d = SmartDashboard.getNumber("Pincher D Gain", 0);
    double iz = SmartDashboard.getNumber("Pincher I Zone", 0);
    double ff = SmartDashboard.getNumber("Pincher Feed Forward", 0);
    double max = SmartDashboard.getNumber("Pincher Max Output", 0);
    double min = SmartDashboard.getNumber("Pincher Min Output", 0);
    double rotations = SmartDashboard.getNumber("Pincher Set Rotations", 0);

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

    SmartDashboard.putNumber("Pincher SetPoint", rotations);
    SmartDashboard.putNumber("Pincher ProcessVariable", m_encoder.getPosition());
  }
}
