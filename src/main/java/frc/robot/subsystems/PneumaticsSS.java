// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSS extends SubsystemBase {
  /** Creates a new PneumaticsSS. */
  public final DoubleSolenoid AlgaeGuide;
  public final DoubleSolenoid Climber;

  public PneumaticsSS() {
    AlgaeGuide =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.PneumaticsConstants.forwardGuideSol,
            Constants.PneumaticsConstants.reverseGuideSol);
    Climber =
            new DoubleSolenoid(
                PneumaticsModuleType.REVPH,
                Constants.PneumaticsConstants.forwardClimbSol,
                Constants.PneumaticsConstants.reverseClimbSol);
    AlgaeGuide.set(Value.kReverse);
    Climber.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    AlgaeStatus();
    ClimberStatus();
    SmartDashboard.putData("Test Status", AlgaeGuide);
  }

  public void ToggleGuide() {
    AlgaeGuide.toggle();
  }

  public void ToggleClimb() {
    Climber.toggle();
  }

  public void AlgaeStatus() {
    AlgaeGuide.get();
  }

  public void ClimberStatus() {
    Climber.get();
  }
}
