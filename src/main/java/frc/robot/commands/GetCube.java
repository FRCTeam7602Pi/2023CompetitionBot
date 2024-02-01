// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PidPincher;

public class GetCube extends Command {

  private final PidPincher m_pincher;

  public GetCube(PidPincher pincher) {
    m_pincher = pincher;
    addRequirements(m_pincher);
  }

  @Override
  public void initialize() {
    m_pincher.clampCube();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_pincher.stop();
    }
  }

  @Override
  public boolean isFinished() {
    System.out.println("CUBE IN POSITION");
    return m_pincher.isInCubePosition();
  }
}
