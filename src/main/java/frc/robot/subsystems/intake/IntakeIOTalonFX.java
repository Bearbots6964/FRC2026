// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO{
    //instantiate intake motor and status signals
    TalonFX intakeMotor;
    TalonFX deployMotor;
    
    //intake motor status signals
    StatusSignal<Angle> intakeMotorPositionRot;
    StatusSignal<AngularVelocity> intakeMotorVelocityRotPerSec;
    StatusSignal<Voltage> intakeMotorAppliedVolts;
    StatusSignal<Current> intakeMotorCurrentAmps;

    //deploy motor status signals
    StatusSignal<Angle> deployMotorPositionRot;
    StatusSignal<AngularVelocity> deployMotorVelocityRotPerSec;
    StatusSignal<Voltage> deployMotorAppliedVolts;
    StatusSignal<Current> deployMotorCurrentAmps;

    //create a new request to reuse for setting voltages and using commands
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeIOTalonFX() {

      //structure for constructor is as follows:
      //1. config
      //2. create motor objects
      //3. apply config to motors, retrying up to 5 times with 0.25s delay
      //4. create status signals
      //5. set update frequency for all status signals
      //6. optimize bus utilization


       //configure the TalonFX for the intake motor, 
       //this is a single motor subsystem so we don't need to worry about multiple motors working together
       var intakeConfig = new TalonFXConfiguration();
       intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeCurrentLimits;
       intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
       intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

       var deployConfig = new TalonFXConfiguration();
        deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.deployCurrentLimits;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       
       
       //create motor object and apply configuration to intake motor, retrying up to 5 times with 0.25s delay
        intakeMotor = new TalonFX(IntakeConstants.IntakeMotorCanID);
        tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));
       
       //create motor object and apply configuration to deploy motor, retrying up to 5 times with 0.25s delay
        deployMotor = new TalonFX(IntakeConstants.deployMotorCanID);
        tryUntilOk(5, () -> deployMotor.getConfigurator().apply(deployConfig, 0.25));

         //create status signals for intake motor
         intakeMotorPositionRot = intakeMotor.getPosition();
         intakeMotorVelocityRotPerSec = intakeMotor.getVelocity();
         intakeMotorAppliedVolts = intakeMotor.getMotorVoltage();
         intakeMotorCurrentAmps = intakeMotor.getSupplyCurrent();

         //create status signals for deploy motor
          deployMotorPositionRot = deployMotor.getPosition();
          deployMotorVelocityRotPerSec = deployMotor.getVelocity();
          deployMotorAppliedVolts = deployMotor.getMotorVoltage();
          deployMotorCurrentAmps = deployMotor.getSupplyCurrent();

          //set update frequency for all status signals to 50Hz, 
       BaseStatusSignal.setUpdateFrequencyForAll(50.0,
             intakeMotorPositionRot,
             intakeMotorVelocityRotPerSec,
             intakeMotorAppliedVolts,
             intakeMotorCurrentAmps,
             deployMotorPositionRot,
             deployMotorVelocityRotPerSec,
             deployMotorAppliedVolts,
             deployMotorCurrentAmps);
             ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
      //end of constructor
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            intakeMotorPositionRot,
            intakeMotorVelocityRotPerSec,
            intakeMotorAppliedVolts,
            intakeMotorCurrentAmps,
            deployMotorPositionRot,
            deployMotorVelocityRotPerSec,
            deployMotorAppliedVolts,
            deployMotorCurrentAmps
            );

            //update the logged inputs with the latest values from the status signals
            inputs.intakeMotorPositionRad = Units.rotationsToRadians(intakeMotorPositionRot.getValueAsDouble());
            inputs.intakeMotorVelocityRadPerSec = Units.rotationsToRadians(intakeMotorVelocityRotPerSec.getValueAsDouble());
            inputs.intakeMotorAppliedVolts = intakeMotorAppliedVolts.getValueAsDouble();
            inputs.intakeMotorCurrentAmps = intakeMotorCurrentAmps.getValueAsDouble();
            inputs.deployMotorPositionRad = Units.rotationsToRadians(deployMotorPositionRot.getValueAsDouble());
            inputs.deployMotorVelocityRadPerSec = Units.rotationsToRadians(deployMotorVelocityRotPerSec.getValueAsDouble());
            inputs.deployMotorAppliedVolts = deployMotorAppliedVolts.getValueAsDouble();
            inputs.deployMotorCurrentAmps = deployMotorCurrentAmps.getValueAsDouble();
    }

   @Override
   public void setIntakeVoltage(double volts) {
     intakeMotor.setControl(voltageRequest.withOutput(volts));
   }
   
   @Override
    public void setDeployVoltage(double volts) {
      deployMotor.setControl(voltageRequest.withOutput(volts));
    }
}
