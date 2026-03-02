// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ClimberConstants {

    public static final class climberMotorConstants {
        public static final int climberMotorCanID = 1;
         //gear ratio is 1:1 brake, 9:1, 5:1 to be implemented
        public static final double climberVoltage = 6.0;
        public static final double climbDegrees = 300.0;
        public static final double descendDegrees = 3.0;

        public static final Slot0Configs climberMotorGains = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0);
   
        public static final CurrentLimitsConfigs climberCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(20.0);
        
        public static final MotorOutputConfigs climberMotorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        




    }


}
