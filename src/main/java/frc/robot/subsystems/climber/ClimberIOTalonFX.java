// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climber.ClimberConstants.climberEncoderConstants;
import frc.robot.subsystems.climber.ClimberConstants.climberMotorConstants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO{

    //instantiate climber motor
    TalonFX climberMotor;
    //instantiate climber encoder
    CANcoder climberEncoder;

    //climber motor status signals
    StatusSignal<Angle> climberMotorPositionRot; //use rotations because getPosition returns rotations
    StatusSignal<AngularVelocity> climberMotorVelocityRadPerSec;
    StatusSignal<Voltage> climberMotorAppliedVolts;
    StatusSignal<Current> climberMotorCurrentAmps;

    //climber encoder status signals
    StatusSignal<Angle> climberEncoderAbsolutePositionRot;

    //create a new request to reuse for setting voltages and using commands
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralOut = new NeutralOut();


    public ClimberIOTalonFX() {
      //structure for constructor is as follows:
      //1. config
      //2. create motor objects
      //3. apply config to motors, retrying up to 5 times with 0.25s delay
      //4. create status signals
      //5. set update frequency for all status signals
      //6. optimize bus utilization
   
       TalonFXConfiguration climberConfig = new TalonFXConfiguration()
            .withMotorOutput(climberMotorConstants.climberMotorOutputConfigs)
            .withCurrentLimits(climberMotorConstants.climberCurrentLimits)
            .withSlot0(climberMotorConstants.climberMotorGains)
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(climberEncoderConstants.climberEncoderCanID)
                .withSensorToMechanismRatio(climberMotorConstants.encoderToMechanismRatio)
                .withRotorToSensorRatio(climberMotorConstants.rotorToEncoderRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = climberEncoderConstants.absoluteSensorDiscontinuityPoint;
        canCoderConfig.MagnetSensor.SensorDirection = climberEncoderConstants.sensorDirection; 
        canCoderConfig.MagnetSensor.MagnetOffset = climberEncoderConstants.magnetOffset;


        //create motor object and apply config
        climberMotor = new TalonFX(climberMotorConstants.climberMotorCanID);
        tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));

        //create encoder object and apply config
        climberEncoder = new CANcoder(climberEncoderConstants.climberEncoderCanID);
        tryUntilOk(5, () -> climberEncoder.getConfigurator().apply(canCoderConfig, 0.25));

        //create status signals
        climberMotorPositionRot = climberMotor.getPosition();
        climberMotorVelocityRadPerSec = climberMotor.getVelocity();
        climberMotorAppliedVolts = climberMotor.getMotorVoltage();
        climberMotorCurrentAmps = climberMotor.getSupplyCurrent();
        climberEncoderAbsolutePositionRot = climberEncoder.getAbsolutePosition();

        //set update frequency for all status signals
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            climberMotorPositionRot,
            climberMotorVelocityRadPerSec,
            climberMotorAppliedVolts,
            climberMotorCurrentAmps,
            climberEncoderAbsolutePositionRot);
        
            //optimize bus utilization
        climberMotor.optimizeBusUtilization();
        climberEncoder.optimizeBusUtilization();

        //end of constructor
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        //refresh the status signals to get the latest data from the motor
        inputs.climberMotorConnected = BaseStatusSignal.refreshAll(
            climberMotorPositionRot,
            climberMotorVelocityRadPerSec,
            climberMotorAppliedVolts,
            climberMotorCurrentAmps
        ).isOK();

        //refresh climber encoder status signals to get the latest data from the encoder
        inputs.climberEncoderConnected = BaseStatusSignal.refreshAll(
            climberEncoderAbsolutePositionRot
        ).isOK();
        inputs.climberEncoderAbsolutePositionRad = Units.rotationsToRadians(climberEncoderAbsolutePositionRot.getValueAsDouble());

        //update the logged inputs with the latest values from the status signals
        inputs.climberMotorPositionRad = Units.rotationsToRadians(climberMotorPositionRot.getValueAsDouble());
        inputs.climberMotorVelocityRadPerSec = Units
           .rotationsPerMinuteToRadiansPerSecond(climberMotorVelocityRadPerSec.getValueAsDouble());
        inputs.climberMotorAppliedVolts = climberMotorAppliedVolts.getValueAsDouble();
        inputs.climberMotorCurrentAmps = climberMotorCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setClimberVoltage(double volts) {
        climberMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stopClimber() {
        climberMotor.setControl(neutralOut);
    }




}
