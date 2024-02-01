// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR_CONTROLLER;
import static frc.robot.Constants.ELEVATOR_SLOW_SCALING;
import static frc.robot.Constants.ELEVATOR_MOVE_VELOCITY;
import static frc.robot.Constants.Elevator.MAX_POSITION;
import static frc.robot.Constants.Elevator.START_POSITION;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorMotor;
  private final RelativeEncoder elevatorEncoder;

  // free form string used to communicate state to dashboard
  private String command = "";

  // used by top and bottom commands; hoping to replace when we
  // move to PID control
  private int targetPosition;

  public Elevator() {
    elevatorMotor = new CANSparkMax(ELEVATOR_CONTROLLER, MotorType.kBrushless);
    elevatorMotor.restoreFactoryDefaults();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    updateDashboard();

    if (isCloseToPosition()) {
      slowDown();
    }
  }

  public void elevatorTop() {
    command = "TOP";
    targetPosition = MAX_POSITION;
    up();
  }

  public void elevatorBottom() {
    command = "BOTTOM";
    targetPosition = START_POSITION;
    down();
  }

  public void moveUp() {
    command = "UP";
    up();
  }

  public void moveDown() {
    command = "DOWN";
    down();
  }

  public void stop() {
    command = "STOP";
    elevatorMotor.stopMotor();
  }

  public boolean isBottom() {
    return (elevatorEncoder.getPosition() - START_POSITION) <= 1.0f;
  }

  public boolean isTop() {
    return (MAX_POSITION - elevatorEncoder.getPosition()) <= 1.0f;
  }

  private void down() {
    if (!isBottom()) {
      elevatorMotor.set(-ELEVATOR_MOVE_VELOCITY);
    } else {
      ignoreCommand();
    }
  }

  private void ignoreCommand() {
    command = command + " (ignored)";
  }

  private boolean isCloseToPosition() {
    double delta = Math.abs(elevatorEncoder.getPosition() - targetPosition);
    return delta < 10.0f;
  }

  private void slowDown() {
    elevatorMotor.set(elevatorMotor.get() / ELEVATOR_SLOW_SCALING);
  }

  private void up() {
    if (!isTop()) {
      elevatorMotor.set(ELEVATOR_MOVE_VELOCITY);
    } else {
      ignoreCommand();
    }
  }

  private void updateDashboard() {
    double position = elevatorEncoder.getPosition();
    SmartDashboard.putNumber("Elevator Encoder Position", position);
    SmartDashboard.putString("Elevator Last Command", command);
  }
}
