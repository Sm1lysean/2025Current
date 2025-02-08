package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class PneumaticsC extends InstantCommand {
  public PneumaticsC(PneumaticsSS subsystem) {
    subsystem = RobotContainer.rc_PneumaticsSS;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.rc_PneumaticsSS.Toggle();
  }
}