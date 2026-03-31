package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.IntakeConsts;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX indexerMotor;
  private final TalonFX indexerFollowerMotor;

  private final TalonFXConfiguration indexerConfiguration;
  private final TalonFXConfiguration followerConfiguration;

  private final StatusSignal<Voltage> indexerVolts;
  private final StatusSignal<Voltage> followerVolts;

  private final Follower followRequest =
      new Follower(
          IntakeConsts.INTAKE_FOLLOWER_ID,
          IntakeConsts.INTAKE_MOTOR_OUTPUT_CONFIGS.Inverted
                  == IntakeConsts.INTAKE_MOTOR_OUTPUT_CONFIGS2.Inverted
              ? MotorAlignmentValue.Aligned
              : MotorAlignmentValue.Opposed);

  private final NeutralOut neutralOut = new NeutralOut();

  public IndexerIOTalonFX() {
    indexerMotor = new TalonFX(IndexerConstants.IndexerConsts.INDEXER_MOTOR_ID);
    indexerFollowerMotor = new TalonFX(IndexerConstants.IndexerConsts.FOLLOWER_INDEXER_MOTOR_ID);

    indexerConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(IndexerConstants.IndexerConsts.INDEXER_LIMITS)
            .withMotorOutput(IndexerConstants.IndexerConsts.INDEXER_MOTOR_OUTPUT_CONFIGS);

    followerConfiguration =
        new TalonFXConfiguration()
            .withMotorOutput(IndexerConstants.IndexerConsts.FOLLOWE_MOTOR_OUTPUT_CONFIGS);

    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfiguration));
    PhoenixUtil.tryUntilOk(
        5, () -> indexerFollowerMotor.getConfigurator().apply(indexerConfiguration));

    indexerVolts = indexerMotor.getMotorVoltage();
    followerVolts = indexerFollowerMotor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(50, indexerVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(4, followerVolts);

    indexerMotor.optimizeBusUtilization();
    indexerFollowerMotor.optimizeBusUtilization();

    indexerFollowerMotor.setControl(followRequest);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerConnected = BaseStatusSignal.refreshAll(indexerVolts).isOK();
    inputs.indexerFollowerConnected = BaseStatusSignal.refreshAll(followerVolts).isOK();

    inputs.indexerVolts = indexerVolts.getValue();
  }

  @Override
  public void runIndexerOpenLoop() {
    indexerMotor.set(IndexerConstants.IndexerConsts.SPEED_PERCENTAGE);
  }

  @Override
  public void stop() {
    indexerMotor.setControl(neutralOut);
  }
}
