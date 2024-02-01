// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PidElevator;

public class ElevatorBottom extends Command {

  private final PidElevator m_elevator;

  public ElevatorBottom(PidElevator elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.elevatorBottom();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.isBottom();
  }
}
