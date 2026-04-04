package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.HoodMotorConstants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX flywheel;
    private final TalonFX hood;

    private final TalonFX shooterFollower1;
    private final TalonFX shooterFollower2;
    private final TalonFX shooterFollower3;

    private final TalonFXConfiguration shooter_configs;
    private final TalonFXConfiguration hood_configs;

    private final TalonFXConfiguration shooterFollower1_configs;
    private final TalonFXConfiguration shooterFollower2_configs;
    private final TalonFXConfiguration shooterFollower3_configs;

    private final StatusSignal<AngularVelocity> shooterSpeed;

    private final StatusSignal<Voltage> shooterFollower1Voltage;
    private final StatusSignal<Voltage> shooterFollower2Voltage;
    private final StatusSignal<Voltage> shooterFollower3Voltage;

    private final StatusSignal<Current> shooterAmps;
    private final StatusSignal<Voltage> shooterVolts;
    private final StatusSignal<AngularAcceleration> shooterAcel;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Voltage> hoodVolts;
    private final StatusSignal<AngularVelocity> hoodSpeed;
    private final MotionMagicVoltage mm = new MotionMagicVoltage(0);
    private final PositionVoltage pv = new PositionVoltage(0);

    private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0).withEnableFOC(false);
    private final VoltageOut shooterVoltageOut = new VoltageOut(0);

    private final Follower followRequest =
        new Follower(
            FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
            MotorAlignmentValue.Aligned);

    private final Follower followRequest2 =
        new Follower(
            FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
            MotorAlignmentValue.Opposed);

    private final Follower followRequest3 =
        new Follower(
            FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
            MotorAlignmentValue.Opposed);

    private final NeutralOut neutralOut = new NeutralOut();

    public ShooterIOTalonFX() {

        flywheel = new TalonFX(FlywheelMotorConstants.FLYWHEEL_MOTOR_ID, new CANBus("Drivebase"));
        shooterFollower1 = new TalonFX(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_1_MOTOR_ID, new CANBus("Drivebase"));
        shooterFollower2 = new TalonFX(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_2_MOTOR_ID, new CANBus("Drivebase"));
        shooterFollower3 = new TalonFX(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_3_MOTOR_ID, new CANBus("Drivebase"));

        hood = new TalonFX(HoodMotorConstants.HOOD_MOTOR_ID, new CANBus("Drivebase"));

        shooter_configs =
            new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
                .withSlot0(FlywheelMotorConstants.FLYWHEEL_GAINS)
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.LEFT_FLYWHEEL_OUTPUT_CONFIGS);

        hood_configs =
            new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
                .withSlot0(HoodMotorConstants.HOOD_GAINS)
                .withCurrentLimits(HoodMotorConstants.HOOD_CURRENT_LIMITS)
                .withMotorOutput(HoodMotorConstants.HOOD_OUTPUT_CONFIGS);
        hood_configs.Feedback.SensorToMechanismRatio = HoodMotorConstants.MOTOR_TO_HOOD_RATIO;

        shooterFollower1_configs =
            new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.LEFT_FLYWHEEL_OUTPUT_CONFIGS);

        shooterFollower2_configs =
            new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.RIGHT_FLYWHEEL_OUTPUT_CONFIGS);

        shooterFollower3_configs =
            new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
                .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FlywheelMotorConstants.RIGHT_FLYWHEEL_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> flywheel.getConfigurator().apply(shooter_configs));
        PhoenixUtil.tryUntilOk(
            5, () -> shooterFollower1.getConfigurator().apply(shooterFollower1_configs));
        PhoenixUtil.tryUntilOk(
            5, () -> shooterFollower2.getConfigurator().apply(shooterFollower2_configs));
        PhoenixUtil.tryUntilOk(
            5, () -> shooterFollower3.getConfigurator().apply(shooterFollower3_configs));
        PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(hood_configs));

        shooterSpeed = flywheel.getVelocity();
        shooterVolts = flywheel.getMotorVoltage();
        shooterAmps = flywheel.getStatorCurrent();
        shooterAcel = flywheel.getAcceleration();

        hoodCurrent = hood.getStatorCurrent();
        hoodVolts = hood.getMotorVoltage();
        hoodPosition = hood.getPosition();
        hoodSpeed = hood.getVelocity();

        hood_configs.MotionMagic.MotionMagicCruiseVelocity = hoodSpeed.getValueAsDouble();

        shooterFollower1Voltage = shooterFollower1.getMotorVoltage();
        shooterFollower2Voltage = shooterFollower2.getMotorVoltage();
        shooterFollower3Voltage = shooterFollower3.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            shooterSpeed,
            shooterVolts,
            shooterAmps,
            shooterAcel,
            hoodPosition,
            hoodSpeed,
            hoodVolts,
            hoodCurrent);

        BaseStatusSignal.setUpdateFrequencyForAll(
            4, shooterFollower1Voltage, shooterFollower2Voltage, shooterFollower3Voltage);

        flywheel.optimizeBusUtilization();
        shooterFollower1.optimizeBusUtilization();
        shooterFollower2.optimizeBusUtilization();
        shooterFollower3.optimizeBusUtilization();

        hood.optimizeBusUtilization();

        shooterFollower1.setControl(followRequest);
        shooterFollower2.setControl(followRequest2);
        shooterFollower3.setControl(followRequest3);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leadMotorConnected =
            BaseStatusSignal.refreshAll(shooterSpeed, shooterVolts, shooterAmps, shooterAcel).isOK();

        inputs.leadMotorVelocity = shooterSpeed.getValue();
        inputs.leadMotorVoltage = shooterVolts.getValue();
        inputs.leadMotorCurrent = shooterAmps.getValue();
        inputs.leadmotorAccel = shooterAcel.getValue();

        inputs.followerMotor1Connected = BaseStatusSignal.refreshAll(shooterFollower1Voltage).isOK();
        inputs.followerMotor2Connected = BaseStatusSignal.refreshAll(shooterFollower2Voltage).isOK();
        inputs.followerMotor3Connected = BaseStatusSignal.refreshAll(shooterFollower3Voltage).isOK();

        inputs.hoodMotorConnected =
            BaseStatusSignal.refreshAll(hoodPosition, hoodSpeed, hoodCurrent, hoodVolts).isOK();

        inputs.hoodMotorAngularVelocity = hoodSpeed.getValue();
        inputs.hoodMotorVoltage = hoodVolts.getValue();
        inputs.hoodMotorCurrent = hoodCurrent.getValue();
        inputs.hoodPosition = hoodPosition.getValue();
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity speed) {
        flywheel.setControl(shooterVelocityVoltage.withVelocity(speed));
        shooterFollower1.setControl(followRequest);
        shooterFollower2.setControl(followRequest2);
        shooterFollower3.setControl(followRequest3);
    }

    @Override
    public void setHoodPosition(Angle position) {
        hood.setControl(pv.withPosition(position));
    }

    @Override
    public void setHoodPIDConstants(double kP, double kD, double kV, double kS) {
        hood_configs.Slot0.kP = kP;
        hood_configs.Slot0.kD = kD;
        hood_configs.Slot0.kV = kV;
        hood_configs.Slot0.kS = kS;
        PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(hood_configs));
    }

    @Override
    public void setFlywheelPIDConstants(double kP, double kD, double kV, double kS) {
        shooter_configs.Slot0.kP = kP;
        shooter_configs.Slot0.kD = kD;
        shooter_configs.Slot0.kV = kV;
        shooter_configs.Slot0.kS = kS;
        PhoenixUtil.tryUntilOk(5, () -> flywheel.getConfigurator().apply(shooter_configs));
    }

    @Override
    public void stopHood() {
        hood.setControl(neutralOut);
    }

    @Override
    public void stopFlywheel() {
        flywheel.setControl(neutralOut);
        shooterFollower1.setControl(neutralOut);
        shooterFollower2.setControl(neutralOut);
        shooterFollower3.setControl(neutralOut);
    }

    public void setFlywheelVoltage(Voltage volts) {
        flywheel.setControl(shooterVoltageOut.withOutput(volts));
        shooterFollower1.setVoltage(volts.in(Units.Volts));
        shooterFollower2.setVoltage(volts.unaryMinus().in(Units.Volts));
        shooterFollower3.setVoltage(volts.unaryMinus().in(Units.Volts));
    }

    public void setHoodVoltage(Voltage volts) {
        hood.setVoltage(volts.in(Units.Volts));
    }
}
