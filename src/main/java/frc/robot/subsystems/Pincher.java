// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pincher extends SubsystemBase {

  private final CANSparkMax pincherMotor;
  private final RelativeEncoder pincherEncoder;

  // used by commands trying to move to a specific position
  private double targetPosition;

  public Pincher() {
    pincherMotor = new CANSparkMax(Constants.PINCHER_CONTROLLER, MotorType.kBrushless);
    pincherMotor.restoreFactoryDefaults();
    pincherMotor.setSmartCurrentLimit(Constants.PINCHER_CURRENT_LIMIT);
    pincherEncoder = pincherMotor.getEncoder();
    pincherEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    double position = pincherEncoder.getPosition();
    SmartDashboard.putNumber("Encoder Position", position);
    SmartDashboard.putNumber("Pincher Angle", position*5);
    SmartDashboard.putNumber("Pincher Target", targetPosition);

    if (isCloseToPosition()) {
      slowDown();
    }
  }

  public void clampCube() {
    System.out.println("Pincher Clamp Cube");
    targetPosition = Constants.Pincher.CUBE_POSITION;
    moveTowardTargetPosition(Constants.PINCHER_MOVE_VELOCITY);
  }

  public void clampStandingCone() {
    System.out.println("Pincher Clamp Standing Cone");
    targetPosition = Constants.Pincher.CONE_POSITION;
    moveTowardTargetPosition(Constants.PINCHER_MOVE_VELOCITY);
  }

  public void close(double velocity) {
    targetPosition = Constants.Pincher.MAX_POSITION;
    System.out.printf("Pincher Closing %.2f\n", velocity);
    if (pincherEncoder.getPosition() >= Constants.Pincher.MAX_POSITION) {
      System.out.println("Hit Close Limit");
    } else {
      move(scale(velocity));
    }
  }

  public boolean isInPosition() {
    return Math.abs(pincherEncoder.getPosition() - targetPosition) < .1f;
  }

  public boolean isPastClosedPosition() {
    System.out.println("Past Closed Position");
    return (pincherEncoder.getPosition() >= Constants.Pincher.MAX_POSITION);
  }

  public void open(double velocity) {
    targetPosition = Constants.Pincher.START_POSITION;
    if (pincherEncoder.getPosition() <= Constants.Pincher.START_POSITION) {
      System.out.println("Hit Open Limit");
    } else {
      move(scale(-velocity));
    }
  }

  public void ready() {
    System.out.println("Pincher Ready Position");
    targetPosition = Constants.Pincher.OPEN_POSITION;
    moveTowardTargetPosition(.1); // Constants.PINCHER_MOVE_VELOCITY);
  }

  public void stop() {
    System.out.println("Pincher Stop");
    pincherMotor.stopMotor();
  }

  private boolean isCloseToPosition() {
    double howClose = Math.abs(pincherEncoder.getPosition() - targetPosition);
    SmartDashboard.putNumber("Pincher Proximity", howClose);
    SmartDashboard.putNumber("Pincher Slosh", Math.abs(pincherMotor.get()) * 8);
    return howClose < (Math.abs(pincherMotor.get()) * 10);
  }

  /**
   * Returns true if current position is past requested position.
   */
  private boolean isPastPosition(double position) {
    return pincherEncoder.getPosition() > position;
  }

  private void move(double velocity) {
    if (!isInPosition()) {
      pincherMotor.set(velocity);
    }
  }

  private void moveTowardTargetPosition(double velocity) {
    move(isPastPosition(targetPosition) ? -velocity : velocity);
  }

  private void slowDown() {
    pincherMotor.set(scale(pincherMotor.get()));
  }

  /**
   * Apply scale from Constants to the given velocity.  This is primarily
   * for manual control since those are expected to be minor adjustments.
   */
  private double scale(double velocity) {
    return velocity / Constants.PINCHER_CONTROL_SCALING;
  }
}
