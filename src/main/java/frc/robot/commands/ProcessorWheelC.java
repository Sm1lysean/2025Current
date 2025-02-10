package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ProcessorWheelC extends Command {

  public ProcessorWheelC(ProcessorWheelSS subsystem) {
    subsystem = RobotContainer.rc_processorwheelSS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.rc_processorwheelSS.Wrist.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void execute() {
    if (RobotContainer.m_operatorController.y().getAsBoolean() == true)
      RobotContainer.rc_processorwheelSS.Spin();
    else if (RobotContainer.m_operatorController.b().getAsBoolean() == true)
      RobotContainer.rc_processorwheelSS.Reverse();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_processorwheelSS.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
