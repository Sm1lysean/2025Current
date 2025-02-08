package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ProcessorWheelSS extends SubsystemBase {

public final TalonSRX Wrist = new TalonSRX(Constants.Wrist);

    public void Spin() {
        Wrist.set(ControlMode.PercentOutput, .5);
    }
    public void Reverse() {
        Wrist.set(ControlMode.PercentOutput, -.5);
    }

    public void Stop() {
        Wrist.set(ControlMode.PercentOutput, 0);
    }

}
