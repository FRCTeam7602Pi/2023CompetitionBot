// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.ARM_TRIGGER_AXIS;
import static frc.robot.Constants.OperatorConstants.ELEVATOR_TRIGGER_AXIS;
import static frc.robot.Constants.OperatorConstants.GAMEPPAD_PORT;
import static frc.robot.Constants.OperatorConstants.JOYSTICK_PORT;
import static frc.robot.Constants.OperatorConstants.PINCHER_CLOSE_AXIS;
import static frc.robot.Constants.OperatorConstants.PINCHER_OPEN_AXIS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClosePinchers;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorBottom;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorTop;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.GetCube;
import frc.robot.commands.OpenPinchers;
import frc.robot.commands.ReadyPinchers;
import frc.robot.commands.RetractArm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PidArm;
import frc.robot.subsystems.PidElevator;
import frc.robot.subsystems.PidPincher;

/**
 * The standard RobotContainer where the bulk of the robot is declared.
 */
public class RobotContainer {
  private final PidArm m_arm = new PidArm();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final PidElevator m_elevator = new PidElevator();
  private final PidPincher m_pincher = new PidPincher();
  private final Lights m_lights = new Lights();

  private final CommandJoystick m_driverController = new CommandJoystick(JOYSTICK_PORT);
  private final CommandXboxController m_elevatorController = new CommandXboxController(GAMEPPAD_PORT);

  public RobotContainer() {
    configureBindings();

    m_driveTrain.setDefaultCommand(
      new Drive(m_driveTrain, () -> -m_driverController.getY(), () -> m_driverController.getTwist())
    );
  }

  /**
   * The joystick is already bound as it is used in the default command for the
   * drive train.  The binding here are for everything but the default drive.
   */
  private void configureBindings() {

    // elevator
    m_elevatorController.axisLessThan(ELEVATOR_TRIGGER_AXIS, -0.5)
      .whileTrue(new ElevatorUp(m_elevator));
    m_elevatorController.axisGreaterThan(ELEVATOR_TRIGGER_AXIS, 0.5)
      .whileTrue(new ElevatorDown(m_elevator));
    m_elevatorController.povUp().onTrue(new ElevatorTop(m_elevator));
    m_elevatorController.povDown().onTrue(new ElevatorBottom(m_elevator));

    // arm
    m_elevatorController.axisLessThan(ARM_TRIGGER_AXIS, -0.5)
      .whileTrue(new ExtendArm(m_arm, () -> m_elevatorController.getRawAxis(ARM_TRIGGER_AXIS)));
    m_elevatorController.axisGreaterThan(ARM_TRIGGER_AXIS, 0.5)
      .whileTrue(new RetractArm(m_arm, () -> m_elevatorController.getRawAxis(ARM_TRIGGER_AXIS)));

    // pincher
    m_elevatorController.axisGreaterThan(PINCHER_OPEN_AXIS, 0.5)
      .whileTrue(new OpenPinchers(m_pincher, () -> m_elevatorController.getRawAxis(PINCHER_OPEN_AXIS)));
    m_elevatorController.axisGreaterThan(PINCHER_CLOSE_AXIS, 0.5)
      .whileTrue(new ClosePinchers(m_pincher, () -> m_elevatorController.getRawAxis(PINCHER_CLOSE_AXIS)));
    // TODO - make this sequential: elevator up, pinchers out, arm extend, elevator down
    m_elevatorController.a().onTrue(new ReadyPinchers(m_pincher));
    m_elevatorController.b().onTrue(new GetCube(m_pincher));

    // lights
    m_elevatorController.x().onTrue(new ConditionalCommand(getLightsOffCommand(), getPurpleLightsCommand(), m_lights::isPurple));
    m_elevatorController.y().onTrue(new ConditionalCommand(getLightsOffCommand(), getYellowLightsCommand(), m_lights::isYellow));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      getMobilityCommand(.5, -.5), 
      getMobilityCommand(5, .5));
  }

  private Command getLightsOffCommand() {
    return Commands.runOnce(m_lights::off, m_lights);
  }

  private Command getPurpleLightsCommand() {
    return Commands.runOnce(m_lights::purple, m_lights);
  }

  private Command getYellowLightsCommand() {
    return Commands.runOnce(m_lights::yellow, m_lights);
  }

  private Command getMobilityCommand(double timeout, double speed) {
    return new RunCommand(() -> {
      m_driveTrain.drive(speed, 0);
    }, m_driveTrain).repeatedly().withTimeout(timeout);
  }
}
