package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.PIDAlgaeSS;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaePIDC extends Command{
  public final PIDAlgaeSS m_algae;
  private final DoubleSupplier m_desiredVelocity;

  public AlgaePIDC(PIDAlgaeSS subsystem, DoubleSupplier desiredVelocity) {
    m_algae = subsystem;
    m_desiredVelocity = desiredVelocity;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_algae.setPosition(m_desiredVelocity.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_algae.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
