// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PidArm;

public class RetractArm extends Command {

  private final PidArm m_arm;
  private final DoubleSupplier m_triggerAxis;

  public RetractArm(PidArm arm, DoubleSupplier triggerAxis) {
    m_arm = arm;
    m_triggerAxis = triggerAxis;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.retract(getTriggerAxis());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Don't want to think about the sign of the trigger axis so getting abs
   * here to use when we want to know value.
   */
  private double getTriggerAxis() {
    return Math.abs(m_triggerAxis.getAsDouble());
  }
}
