// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
@AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorPositionRad = 0.0;
        public double intakeMotorVelocityRadPerSec = 0.0;
        public double intakeMotorAppliedVolts = 0.0;
        public double intakeMotorCurrentAmps = 0.0;

        public double deployMotorPositionRad = 0.0;
        public double deployMotorVelocityRadPerSec = 0.0;
        public double deployMotorAppliedVolts = 0.0;
        public double deployMotorCurrentAmps = 0.0;
    }
    
    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    /** Run the intake motor at the specified voltage. */
    public default void setIntakeVoltage(double volts) {}

    /** Run the deploy motor at the specified voltage. */
    public default void setDeployVoltage(double volts) {}

}
