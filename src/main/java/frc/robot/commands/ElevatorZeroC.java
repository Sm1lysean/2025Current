// WTF does this do\?
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDElevSS1;
import frc.robot.RobotContainer;

public class ElevatorZeroC extends Command {

  public ElevatorZeroC(PIDElevSS1 subsystem) {
    // subsystem = RobotContainer.rc_pidSS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.rc_pidElevSS1.Zero();
    RobotContainer.rc_pidElevSS2.Zero();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_pidElevSS1.MotorStop();
    RobotContainer.rc_pidElevSS2.MotorStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
