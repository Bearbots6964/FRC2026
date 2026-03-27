package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.Constants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooter;
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

  private final StatusSignal<Voltage> shooterFollower1Speed;
  private final StatusSignal<Voltage> shooterFollower2Speed;
  private final StatusSignal<Voltage> shooterFollower3Speed;

  private final StatusSignal<Current> shooterAmps;
  private final StatusSignal<Voltage> shooterVolts;
  private final StatusSignal<AngularAcceleration> shooterAcel;

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Current> hoodCurrent;
  private final StatusSignal<Voltage> hoodVolts;
  private final StatusSignal<AngularVelocity> hoodSpeed;
  private final MotionMagicVoltage mm = new MotionMagicVoltage(0);

  private final VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);

  private final Follower followRequest =
      new Follower(
          Constants.mainShooterConstants.SHOOTER_FOLLOW1_MOTOR_ID,
          Constants.mainShooterConstants.SHOOTER_FOLLWER_CONFIGS.Inverted
                  == Constants.mainShooterConstants.SHOOTER_OUTPUT_CONFIGS.Inverted
              ? MotorAlignmentValue.Aligned
              : MotorAlignmentValue.Opposed);

  private final Follower followRequest2 =
      new Follower(
          Constants.mainShooterConstants.SHOOTER_FOLLO2_MOTOR_ID,
          Constants.mainShooterConstants.SHOOTER_FOLLWER_CONFIGS.Inverted
                  == Constants.mainShooterConstants.SHOOTER_OUTPUT_CONFIGS.Inverted
              ? MotorAlignmentValue.Aligned
              : MotorAlignmentValue.Opposed);

  private final Follower followRequest3 =
      new Follower(
          Constants.mainShooterConstants.SHOOTER_FOLLOW3_MOTOR_ID,
          Constants.mainShooterConstants.SHOOTER_FOLLWER_CONFIGS.Inverted
                  == Constants.mainShooterConstants.SHOOTER_OUTPUT_CONFIGS.Inverted
              ? MotorAlignmentValue.Aligned
              : MotorAlignmentValue.Opposed);

  private final NeutralOut neutralOut = new NeutralOut();

  public ShooterIOTalonFX() {

    shooter = new TalonFX(Constants.mainShooterConstants.SHOOTER_LEAD_MOTOR_ID);
    shooterFollower1 = new TalonFX(Constants.mainShooterConstants.SHOOTER_FOLLOW1_MOTOR_ID);
    shooterFollower2 = new TalonFX(Constants.mainShooterConstants.SHOOTER_FOLLO2_MOTOR_ID);
    shooterFollower3 = new TalonFX(Constants.mainShooterConstants.SHOOTER_FOLLOW3_MOTOR_ID);

    hood = new TalonFX(Constants.hoodConstants.HOOD_MOTOR_ID);

    shooter_configs =
        new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
            .withSlot0(Constants.mainShooterConstants.SHOOTER_GAINS)
            .withCurrentLimits(Constants.mainShooterConstants.SHOOTER_CONFIG)
            .withMotorOutput(Constants.mainShooterConstants.SHOOTER_OUTPUT_CONFIGS);

    hood_configs =
        new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
            .withSlot0(Constants.hoodConstants.HOOD_GAINS)
            .withCurrentLimits(Constants.hoodConstants.HOOD_CONFIG)
            .withMotorOutput(Constants.mainShooterConstants.SHOOTER_OUTPUT_CONFIGS);

    shooterFollower1_configs =
        new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
            .withCurrentLimits(Constants.mainShooterConstants.SHOOTER_CONFIG)
            .withMotorOutput(Constants.mainShooterConstants.SHOOTER2_OUTPUT_CONFIGS);

    shooterFollower2_configs =
        new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
            .withCurrentLimits(Constants.mainShooterConstants.SHOOTER_CONFIG)
            .withMotorOutput(Constants.mainShooterConstants.SHOOTER3_OUTPUT_CONFIGS);

    shooterFollower3_configs =
        new TalonFXConfiguration() // ASK PARKER ABOUT OUTPUT
            .withCurrentLimits(Constants.mainShooterConstants.SHOOTER_CONFIG)
            .withMotorOutput(Constants.mainShooterConstants.SHOOTER4_OUTPUT_CONFIGS);

    PhoenixUtil.tryUntilOk(5, () -> shooter.getConfigurator().apply(shooter_configs));
    PhoenixUtil.tryUntilOk(
        5, () -> shooterFollower1.getConfigurator().apply(shooterFollower1_configs));
    PhoenixUtil.tryUntilOk(
        5, () -> shooterFollower2.getConfigurator().apply(shooterFollower2_configs));
    PhoenixUtil.tryUntilOk(
        5, () -> shooterFollower3.getConfigurator().apply(shooterFollower3_configs));
    PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(hood_configs));

    shooterSpeed = shooter.getVelocity();
    shooterVolts = shooter.getMotorVoltage();
    shooterAmps = shooter.getStatorCurrent();
    shooterAcel = shooter.getAcceleration();

    hoodCurrent = hood.getStatorCurrent();
    hoodVolts = hood.getMotorVoltage();
    hoodPosition = hood.getPosition();
    hoodSpeed = hood.getVelocity();

    hood_configs.MotionMagic.MotionMagicCruiseVelocity = hoodSpeed.getValueAsDouble();

    shooterFollower1Speed = shooterFollower1.getMotorVoltage();
    shooterFollower2Speed = shooterFollower2.getMotorVoltage();
    shooterFollower3Speed = shooterFollower3.getMotorVoltage();

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
        4, shooterFollower1Speed, shooterFollower2Speed, shooterFollower3Speed);

    shooter.optimizeBusUtilization();
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

    inputs.followerMotor1Connected = BaseStatusSignal.refreshAll(shooterFollower1Speed).isOK();
    inputs.followerMotor2Connected = BaseStatusSignal.refreshAll(shooterFollower2Speed).isOK();
    inputs.followerMotor3Connected = BaseStatusSignal.refreshAll(shooterFollower3Speed).isOK();

    inputs.hoodMotorConnected =
        BaseStatusSignal.refreshAll(hoodPosition, hoodSpeed, hoodCurrent, hoodVolts).isOK();

    inputs.hoodMotorAngularVelocity = hoodSpeed.getValue();
    inputs.hoodMotorVoltage = hoodVolts.getValue();
    inputs.hoodMotorCurrent = hoodCurrent.getValue();
    inputs.hoodPosition = hoodPosition.getValue();
  }

  @Override
  public void startShooter(AngularVelocity speed) {
    shooter.setControl(shooterVelocityVoltage.withVelocity(speed));
  }

  @Override
  public void setHoodPosition(Angle position) {
    hood.setControl(mm.withPosition(position));
  }

  @Override
  public void setHoodPIDCOnstants(double kP, double kD, double kV, double kS) {
    hood_configs.Slot0.kP = kP;
    hood_configs.Slot0.kD = kD;
    hood_configs.Slot0.kV = kV;
    hood_configs.Slot0.kS = kS;
    PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(hood_configs));
  }

  @Override
  public void setShooterPIDCOnstants(double kP, double kD, double kV, double kS) {
    shooter_configs.Slot0.kP = kP;
    shooter_configs.Slot0.kD = kD;
    shooter_configs.Slot0.kV = kV;
    shooter_configs.Slot0.kS = kS;
    PhoenixUtil.tryUntilOk(5, () -> shooter.getConfigurator().apply(shooter_configs));
  }

  @Override
  public void stopHood() {
    hood.setControl(neutralOut);
  }

  public void stopShooter() {
    shooter.setControl(neutralOut);
  }
}
