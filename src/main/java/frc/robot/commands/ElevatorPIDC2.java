// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import java.util.function.DoubleSupplier;

public class ElevatorPIDC2 extends Command {
  public final PIDElevSS2 m_elev;
  private final DoubleSupplier m_desiredVelocity;

  public ElevatorPIDC2(PIDElevSS2 subsystem, DoubleSupplier desiredVelocity) {
    m_elev = subsystem;
    m_desiredVelocity = desiredVelocity;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elev.setPosition(m_desiredVelocity.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_elev.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
