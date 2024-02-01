// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public static final boolean USE_OMNIS = false;

  private final CANSparkMax driveLeft;
  private final CANSparkMax driveRight;
  private final DifferentialDrive drive;

  private CANSparkMax omniLeft;
  private CANSparkMax omniRight;
  private DifferentialDrive omniDrive;

  public DriveTrain() {
    driveLeft = new CANSparkMax(Constants.DRIVE_LEFT_CONTROLLER, MotorType.kBrushless);
    driveRight = new CANSparkMax(Constants.DRIVE_RIGHT_CONTROLLER, MotorType.kBrushless);
    omniLeft = new CANSparkMax(Constants.OMNI_LEFT_CONTROLLER, MotorType.kBrushless);
    omniRight = new CANSparkMax(Constants.OMNI_RIGHT_CONTROLLER, MotorType.kBrushless);

    driveLeft.restoreFactoryDefaults();
    driveLeft.restoreFactoryDefaults();
    omniLeft.restoreFactoryDefaults();
    omniRight.restoreFactoryDefaults();

    // driveLeft.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    // driveRight.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    // omniLeft.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
    // omniRight.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);

    driveRight.setInverted(true);
    omniRight.setInverted(true);

    drive = new DifferentialDrive(driveLeft, driveRight);
    omniDrive = new DifferentialDrive(omniLeft, omniRight);

    // disabling motor safety since this is drive train and user controls
    // don't always change 10 times per second
    drive.setSafetyEnabled(false);
    omniDrive.setSafetyEnabled(false);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void drive(double forward, double rotation) {
    drive.arcadeDrive(forward * 0.8, -rotation * 0.85);
    if(Math.abs(forward) > .1 || Math.abs(rotation) > .1) {
      System.out.format("DRIVING %.2f by %.2f\n", forward, rotation);
    }
    if(USE_OMNIS && (Math.abs(forward) > .7)) {
      omniDrive.arcadeDrive(forward, rotation);
      System.out.format("BOOSTING with OMNI %.2f by %.2f\n", forward, rotation);
    } else {
      // note that it is important that the drive train motor controllers
      // be configured for coasting when off
      omniDrive.arcadeDrive(0, 0);
    }
  }
}
