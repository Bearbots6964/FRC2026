// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface climberIO {
    @AutoLog
        public static class climberIOInputs{
            public double climberMotorPositionRad = 0.0;
            public double climberMotorAppliedVolts = 0.0;
            public double climberMotorCurrentAmps = 0.0;
        }
     
    public default void updateInputs(climberIOInputs inputs) {}
    public default void setClimberVoltage(double volts) {}

    }
