// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Lights.COLOR_PURPLE;
import static frc.robot.Constants.Lights.COLOR_YELLOW;
import static frc.robot.Constants.Lights.LED_COUNT;
import static frc.robot.Constants.Lights.PORT;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  private final AddressableLED m_leds;
  private final AddressableLEDBuffer m_ledBuffer;

  private State m_state;

  public Lights() {
    m_leds = new AddressableLED(PORT);
    m_ledBuffer = new AddressableLEDBuffer(LED_COUNT);
    m_leds.setLength(m_ledBuffer.getLength());
    m_leds.setData(m_ledBuffer);
    m_leds.start();
    m_state = State.OFF;
  }

  @Override
  public void periodic() {}

  public boolean isPurple() {
    return m_state == State.PURPLE;
  }

  public boolean isYellow() {
    return m_state == State.YELLOW;
  }

  public void off() {
    setSolidColor(new int[] {0, 0, 0});
    m_state = State.OFF;
  }

  public void purple() {
    setSolidColor(COLOR_PURPLE);
    m_state = State.PURPLE;
  }

  public void yellow() {
    setSolidColor(COLOR_YELLOW);
    m_state = State.YELLOW;
  }

  private void setSolidColor(int[] color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
    m_leds.setData(m_ledBuffer);
  }

  /**
   * Keeping track of state so we can toggle lights on / off with a single
   * button instead of using a separate button for off.
   */
  enum State {
    OFF,
    PURPLE,
    YELLOW;
  }
}
