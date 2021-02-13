/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;

public class StartIntake extends InstantCommand {

  private Intake_Subsystem m_intake;
  public StartIntake(Intake_Subsystem m_intake) {
    addRequirements(m_intake);
    this.m_intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.lowerIntake();
    m_intake.intakeOn(0.5);
  }

}
