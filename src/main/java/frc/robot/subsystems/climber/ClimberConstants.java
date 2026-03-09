// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class ClimberConstants {

    public static final class climberMotorConstants {
        public static final int climberMotorCanID = 5;
         
        //gear ratio
        public static final double encoderToMechanismRatio = 2.0;
        public static final double rotorToEncoderRatio = 45.0/2.0;
        
        public static final double climberVoltage = 6.0;
        public static final Angle climbAngle = Degrees.of(90.0);
        public static final Angle descendAngle = Degrees.of(0.0);
        public static final Angle climbTolerance = Degrees.of(0.5);
        public static final double climbTimeoutSeconds = 10.0; //safety timeout — tune based on how long a full climb takes

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

    //climber encoder constants, including CAN ID, magnet offset, and sensor direction
    public static final class climberEncoderConstants {
        public static final double absoluteSensorDiscontinuityPoint = 0.5;
        public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final double magnetOffset = 0.5;
        
        public static final int climberEncoderCanID = 6;
    }


}
