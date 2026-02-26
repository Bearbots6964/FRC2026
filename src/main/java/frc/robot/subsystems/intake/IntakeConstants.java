// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class IntakeConstants {

 public static final class intakeMotorConstants {
    public static final int intakeMotorCanID = 1;
    public static final double gearRatio = 5.0; //gear ratio is 5:1
    public static final double intakeVoltage = 6.0;
    public static final Slot0Configs intakeMotorGains = new Slot0Configs()
            .withKP(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0);
   
            public static final CurrentLimitsConfigs intakeCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20.0);
    
            public static final MotorOutputConfigs intakeMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
  }


 public static final class deployMotorConstants {
    public static final int deployMotorCanID = 2;
    //gear ratio is to be implemented, 4:1, 3:1, 3:1
    public static final double deployVoltage = 6.0;
    public static final Slot0Configs deployMotorGains = new Slot0Configs()
            .withKP(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0);

            public static final CurrentLimitsConfigs deployCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(20.0);
    
            public static final MotorOutputConfigs deployMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
  }

}
