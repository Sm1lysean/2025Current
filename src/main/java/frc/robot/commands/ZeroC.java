package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.PIDSS;

public class ZeroC extends Command{
    
    public ZeroC(PIDSS subsystem) {
        subsystem = RobotContainer.rc_pidSS;
        addRequirements(subsystem);
      }
    
      @Override
      public void initialize() {}
    
      @Override
      public void execute() {
        RobotContainer.rc_pidSS.Zero();
      }
    
      @Override
      public void end(boolean interrupted) {
        RobotContainer.rc_pidSS.MotorStop();
      }
    
      @Override
      public boolean isFinished() {
        return false;
    }
}