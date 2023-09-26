// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LineSensor extends SubsystemBase {
  private AnalogInput leftSensor = new AnalogInput(0);
  private AnalogInput rightSensor = new AnalogInput(1);
  private AnalogInput test = new AnalogInput(2);
  
  /** Creates a new LineSensor. */
  public LineSensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Left: " + leftSensor.getVoltage() + ", Right: " + rightSensor.getVoltage());
  }
}
