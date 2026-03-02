// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climber.ClimberConstants.climberMotorConstants;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO{

    //instantiate climber motor and status signals
    TalonFX climberMotor;

    //climber motor status signals
    StatusSignal<Angle> climberMotorPositionRot; //use rotations because getPosition returns rotations
    StatusSignal<AngularVelocity> climberMotorVelocityRadPerSec;
    StatusSignal<Voltage> climberMotorAppliedVolts;
    StatusSignal<Current> climberMotorCurrentAmps;

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
   
       var climberConfig = new TalonFXConfiguration()
            .withMotorOutput(climberMotorConstants.climberMotorOutputConfigs)
            .withCurrentLimits(climberMotorConstants.climberCurrentLimits)
            .withSlot0(climberMotorConstants.climberMotorGains);
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        //create motor object and apply config
        climberMotor = new TalonFX(climberMotorConstants.climberMotorCanID);
        tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));

        //create status signals
        climberMotorPositionRot = climberMotor.getPosition();
        climberMotorVelocityRadPerSec = climberMotor.getVelocity();
        climberMotorAppliedVolts = climberMotor.getMotorVoltage();
        climberMotorCurrentAmps = climberMotor.getSupplyCurrent();

        //set update frequency for all status signals
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
            climberMotorPositionRot,
            climberMotorVelocityRadPerSec,
            climberMotorAppliedVolts,
            climberMotorCurrentAmps);
        
            //optimize bus utilization
        climberMotor.optimizeBusUtilization();

        //end of constructor
    }

    @Override
    public void updateInputs(climberIOInputs inputs) {
        //refresh the status signals to get the latest data from the motor
        inputs.climberMotorConnected = BaseStatusSignal.refreshAll(
            climberMotorPositionRot,
            climberMotorVelocityRadPerSec,
            climberMotorAppliedVolts,
            climberMotorCurrentAmps
        ).isOK();
    
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
