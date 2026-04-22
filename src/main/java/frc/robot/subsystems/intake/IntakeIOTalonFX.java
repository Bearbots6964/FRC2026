// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.deployMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants.intakeMotorConstants;

/**
 * Add your docs here.
 */
public class IntakeIOTalonFX implements IntakeIO {

    //instantiate intake motor and status signals
    private final TalonFX intakeMotor;
    private final TalonFX intakeFollowerMotor;
    private final TalonFX deployMotor;

    //intake motor status signals
    StatusSignal<Angle> intakeMotorPositionRot;
    StatusSignal<AngularVelocity> intakeMotorVelocityRotPerSec;
    StatusSignal<Voltage> intakeMotorAppliedVolts;
    StatusSignal<Current> intakeMotorCurrentAmps;

    StatusSignal<Voltage> intakeMotorFollowerVoltage;

    //deploy motor status signals
    StatusSignal<Angle> deployMotorPositionRot;
    StatusSignal<AngularVelocity> deployMotorVelocityRotPerSec;
    StatusSignal<Voltage> deployMotorAppliedVolts;
    StatusSignal<Current> deployMotorCurrentAmps;

    //create a new request to reuse for setting voltages and using commands
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage  velocityRequest = new VelocityVoltage(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOTalonFX() {

        //structure for constructor is as follows:
        //1. config
        //2. create motor objects
        //3. apply config to motors, retrying up to 5 times with 0.25s delay
        //4. create status signals
        //5. set update frequency for all status signals
        //6. optimize bus utilization

        //configure the TalonFX for the intake motor,
        var intakeConfig = new TalonFXConfiguration()
            .withMotorOutput(intakeMotorConstants.intakeMotorOutputConfigs)
            .withCurrentLimits(intakeMotorConstants.intakeCurrentLimits)
            .withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod(0.5))
            .withSlot0(intakeMotorConstants.intakeMotorGains);
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.Feedback.SensorToMechanismRatio = 3.0;


        var deployConfig = new TalonFXConfiguration()
            .withMotorOutput(deployMotorConstants.deployMotorOutputConfigs)
            .withCurrentLimits(deployMotorConstants.deployCurrentLimits)
            .withSlot0(deployMotorConstants.deployMotorGains)
            .withSoftwareLimitSwitch(deployMotorConstants.deploySoftwareLimitSwitchConfigs)
            .withMotionMagic(deployMotorConstants.MOTION_MAGIC_CONFIGS)
            .withFeedback(deployMotorConstants.deployFeedbackConfigs);
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //create motor object and apply configuration to intake motor, retrying up to 5 times with 0.25s delay
        intakeMotor = new TalonFX(intakeMotorConstants.intakeMotorCanID);
        tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

        intakeFollowerMotor = new TalonFX(intakeMotorConstants.intakeFollowerMotorCanID);
        tryUntilOk(5, () -> intakeFollowerMotor.getConfigurator().apply(intakeConfig, 0.25));

        intakeMotorFollowerVoltage = intakeFollowerMotor.getMotorVoltage();

        //create motor object and apply configuration to deploy motor, retrying up to 5 times with 0.25s delay
        deployMotor = new TalonFX(deployMotorConstants.deployMotorCanID);
        tryUntilOk(5, () -> deployMotor.getConfigurator().apply(deployConfig, 0.25));

        // set the deploy motor to the retracted position on startup to ensure it starts in a known state, and to prevent it from trying to move to an out-of-bounds position
        deployMotor.setPosition(IntakeConstants.START_ANGLE);

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

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, intakeMotorFollowerVoltage);

        ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
        ParentDevice.optimizeBusUtilizationForAll(deployMotor);
        ParentDevice.optimizeBusUtilizationForAll(intakeFollowerMotor);

        intakeFollowerMotor.setControl(new Follower(intakeMotorConstants.intakeMotorCanID, MotorAlignmentValue.Opposed));

        //end of constructor
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = BaseStatusSignal.refreshAll(
            intakeMotorPositionRot,
            intakeMotorVelocityRotPerSec,
            intakeMotorAppliedVolts,
            intakeMotorCurrentAmps
        ).isOK();

        inputs.deployMotorConnected = BaseStatusSignal.refreshAll(
            deployMotorPositionRot,
            deployMotorVelocityRotPerSec,
            deployMotorAppliedVolts,
            deployMotorCurrentAmps
        ).isOK();
        inputs.intakeFollowerMotorConnected = BaseStatusSignal.refreshAll(intakeMotorFollowerVoltage).isOK();

        //update the logged inputs with the latest values from the status signals
        inputs.intakeMotorPositionDegrees =
            intakeMotorPositionRot.getValueAsDouble() * 360.0;
        inputs.intakeMotorVelocityDegreesPerSec =
            intakeMotorVelocityRotPerSec.getValueAsDouble() * 360.0;
        inputs.intakeMotorAppliedVolts = intakeMotorAppliedVolts.getValueAsDouble();
        inputs.intakeMotorCurrentAmps = intakeMotorCurrentAmps.getValueAsDouble();
        inputs.deployMotorPositionDegrees =
            deployMotorPositionRot.getValueAsDouble() * 360.0;
        inputs.deployMotorVelocityDegreesPerSec =
            deployMotorVelocityRotPerSec.getValueAsDouble() * 360.0;
        inputs.deployMotorAppliedVolts = deployMotorAppliedVolts.getValueAsDouble();
        inputs.deployMotorCurrentAmps = deployMotorCurrentAmps.getValueAsDouble();
    }

    @Override
    public void setIntakeVoltage(Voltage volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setIntakeMotorVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setDeployVoltage(Voltage volts) {
        deployMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setDeployPosition(Angle angle) {
        deployMotor.setControl(motionMagicRequest.withPosition(angle).withSlot(0));
    }

    @Override
    public void setDeployPositionGainSlotTwo(Angle angle) {
        deployMotor.setControl(positionRequest.withPosition(angle).withSlot(1));
    }

    @Override
    public void stopIntake() {
        intakeMotor.setControl(neutralOut);
    }

    @Override
    public void stopDeploy() {
        deployMotor.setControl(neutralOut);
    }

}