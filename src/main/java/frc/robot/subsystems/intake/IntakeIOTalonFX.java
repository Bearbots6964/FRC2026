package frc.robot.subsystems.intake;

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
import frc.robot.subsystems.intake.IntakeConstants.IntakeConsts;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX leadIntake;
  private final TalonFX followerIntake;

  private final TalonFX deployMotor;

  private final TalonFXConfiguration leadConfiguration;
  private final TalonFXConfiguration followerConfiguration;

  private final TalonFXConfiguration deployConfiguration;

  private final StatusSignal<AngularVelocity> intakeSpeed;
  private final StatusSignal<Voltage> intakeVolts;
  private final StatusSignal<Current> intakeAmps;
  private final StatusSignal<AngularAcceleration> intakeAccel;

  private final StatusSignal<Voltage> followerVolts;

  private final StatusSignal<AngularVelocity> deploySpeed;
  private final StatusSignal<Voltage> deployVolts;
  private final StatusSignal<Current> deployAmps;
  private final StatusSignal<AngularAcceleration> deployAccel;
  private final StatusSignal<Angle> deployPos;

  private final MotionMagicVoltage mm = new MotionMagicVoltage(0.0);

  private final VelocityVoltage vv = new VelocityVoltage(0);

  private final Follower followRequest =
      new Follower(
          IntakeConsts.INTAKE_FOLLOWER_ID,
          IntakeConsts.INTAKE_MOTOR_OUTPUT_CONFIGS.Inverted
                  == IntakeConsts.INTAKE_MOTOR_OUTPUT_CONFIGS2.Inverted
              ? MotorAlignmentValue.Aligned
              : MotorAlignmentValue.Opposed);

  private final NeutralOut neutralOut = new NeutralOut();

  public IntakeIOTalonFX() {
    leadIntake = new TalonFX(IntakeConsts.INTAKE_LEAD_ID);
    followerIntake = new TalonFX(IntakeConsts.INTAKE_FOLLOWER_ID);

    deployMotor = new TalonFX(IntakeConstants.intakeDeploy.INTAKE_DEPLOY_ID);

    leadConfiguration =
        new TalonFXConfiguration()
            .withSlot0(IntakeConstants.IntakeConsts.INTAKE_CONFIGS)
            .withCurrentLimits(IntakeConstants.IntakeConsts.INTAKE_CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(IntakeConstants.IntakeConsts.INTAKE_MOTOR_OUTPUT_CONFIGS);

    followerConfiguration =
        new TalonFXConfiguration()
            .withMotorOutput(IntakeConstants.IntakeConsts.INTAKE_MOTOR_OUTPUT_CONFIGS2);

    deployConfiguration =
        new TalonFXConfiguration()
            .withSlot0(IntakeConstants.intakeDeploy.INTAKE_DEPLOY_SLOT0_CONFIGS)
            .withCurrentLimits(IntakeConstants.intakeDeploy.INTAKE_DEPLOY_CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(IntakeConstants.intakeDeploy.INTAKE_DEPLOY_OUTPUT_CONFIGS);

    PhoenixUtil.tryUntilOk(5, () -> leadIntake.getConfigurator().apply(leadConfiguration));
    PhoenixUtil.tryUntilOk(5, () -> deployMotor.getConfigurator().apply(deployConfiguration));
    PhoenixUtil.tryUntilOk(5, () -> followerIntake.getConfigurator().apply(followerConfiguration));

    intakeSpeed = leadIntake.getVelocity();
    intakeVolts = leadIntake.getMotorVoltage();
    intakeAmps = leadIntake.getStatorCurrent();
    intakeAccel = leadIntake.getAcceleration();

    followerVolts = followerIntake.getMotorVoltage();

    deploySpeed = deployMotor.getVelocity();
    deployVolts = deployMotor.getMotorVoltage();
    deployAmps = deployMotor.getStatorCurrent();
    deployAccel = deployMotor.getAcceleration();
    deployPos = deployMotor.getPosition();

    deployConfiguration.MotionMagic.MotionMagicCruiseVelocity = deploySpeed.getValueAsDouble();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeSpeed,
        intakeVolts,
        intakeAmps,
        intakeAccel,
        deploySpeed,
        deployVolts,
        deployAmps,
        deployAccel,
        deployPos);

    BaseStatusSignal.setUpdateFrequencyForAll(4, followerVolts);

    leadIntake.optimizeBusUtilization();

    followerIntake.optimizeBusUtilization();

    deployMotor.optimizeBusUtilization();

    followerIntake.setControl(followRequest);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeConnected =
        BaseStatusSignal.refreshAll(intakeSpeed, intakeVolts, intakeAmps, intakeAccel).isOK();

    inputs.intakeSpeed = intakeSpeed.getValue();
    inputs.intakeVoltage = intakeVolts.getValue();
    inputs.intakeAccel = intakeAccel.getValue();
    inputs.intakeCurrent = intakeAmps.getValue();

    inputs.intake2Connected = BaseStatusSignal.refreshAll(followerVolts).isOK();

    inputs.intakeDeployConnected =
        BaseStatusSignal.refreshAll(deploySpeed, deployVolts, deployAmps, deployAccel, deployPos)
            .isOK();

    inputs.intakeDeployVelocity = deploySpeed.getValue();
    inputs.deployVoltage = deployVolts.getValue();
    inputs.intakeDepoloyAcceleration = deployAccel.getValue();
    inputs.intakeDeployCurrent = deployAmps.getValue();
    inputs.deployPosition = deployPos.getValue();
  }

  @Override
  public void runIntakeClosedLoop(AngularVelocity speed) {
    leadIntake.setControl(vv.withVelocity(speed));
  }

  @Override
  public void deployToPosition(Angle pos) {
    deployMotor.setControl(mm.withPosition(pos));
  }

  @Override
  public void setIntakePID(double kP, double kD, double kV, double kS) {
    leadConfiguration.Slot0.kP = kP;
    leadConfiguration.Slot0.kD = kD;
    leadConfiguration.Slot0.kV = kV;
    leadConfiguration.Slot0.kS = kS;
    PhoenixUtil.tryUntilOk(5, () -> leadIntake.getConfigurator().apply(leadConfiguration));
  }

  @Override
  public void setDeployPID(double kP, double kD, double kV, double kS) {
    deployConfiguration.Slot0.kP = kP;
    deployConfiguration.Slot0.kD = kD;
    deployConfiguration.Slot0.kV = kV;
    deployConfiguration.Slot0.kS = kS;
    PhoenixUtil.tryUntilOk(5, () -> deployMotor.getConfigurator().apply(deployConfiguration));
  }

  @Override
  public void stopDeploy() {
    deployMotor.setControl(neutralOut);
  }

  @Override 
  public void rectractIntake(Angle pos) {
    deployMotor.setControl(mm.withPosition(pos));
  }

  @Override
  public void stopIntake() {
    leadIntake.setControl(neutralOut);
  }
}
