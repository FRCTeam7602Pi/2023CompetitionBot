// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PidPincher;

/**
 * Default control command for the pinchers.  The plan is that the left
 * trigger on the gamepad will open the pinchers and the right trigger
 * will clamp the pinchers.  When the trigger is pressed the pinchers
 * should move and then stop moving when the trigger is released.
 */
public class OpenPinchers extends Command {

  private final PidPincher m_pincher;
  private final DoubleSupplier m_triggerAxis;

  public OpenPinchers(PidPincher pincher, DoubleSupplier triggerAxis) {
    m_pincher = pincher;
    m_triggerAxis = triggerAxis;
    addRequirements(m_pincher);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_pincher.open(Math.abs(m_triggerAxis.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_pincher.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
