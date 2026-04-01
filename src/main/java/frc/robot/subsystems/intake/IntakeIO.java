// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
@AutoLog
    public static class IntakeIOInputs {
        public boolean intakeMotorConnected = false;
        public boolean intakeFollowerMotorConnected = false;
        public boolean deployMotorConnected = false;
        
        public double intakeMotorPositionDegrees = 0.0;
        public double intakeMotorVelocityDegreesPerSec = 0.0;
        public double intakeMotorAppliedVolts = 0.0;
        public double intakeMotorCurrentAmps = 0.0;

        public double deployMotorPositionDegrees = 0.0;
        public double deployMotorVelocityDegreesPerSec = 0.0;
        public double deployMotorAppliedVolts = 0.0;
        public double deployMotorCurrentAmps = 0.0;
    }
    
    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    /** Run the intake motor at the specified voltage. */
    public default void setIntakeVoltage(Voltage volts) {}

    /** Run the deploy motor at the specified voltage. */
    public default void setDeployVoltage(Voltage volts) {}

    public default void setDeployPosition(Angle angle) {}
    public default void setDeployPositionGainSlotTwo(Angle angle) {}

    /** Stop the intake motor. */
    public default void stopIntake() {}

    /** Stop the deploy motor. */
    public default void stopDeploy() {}

}
