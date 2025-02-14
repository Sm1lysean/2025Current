package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class MotorsSS extends SubsystemBase{
    public final SparkMax Algae = new SparkMax(12, MotorType.kBrushless);
    public final TalonSRX Coral = new TalonSRX(2);

    public void AlgaeForward() {
        Coral.set(ControlMode.PercentOutput, .5);
    }

    public void AlgaeReverse() {
        Coral.set(ControlMode.PercentOutput, -0.5);
    }

    public void AlgaeStop() {
        Coral.set(ControlMode.PercentOutput, 0);
    }

    public void Coral(){
        Algae.set(0.5);
    }

    public void CoralStop() {
        Algae.set(0);
    }
}
