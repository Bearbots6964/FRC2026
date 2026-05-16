package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerConstants.TalonFXConstants;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

    // Motor
    private final TalonFX motor;
    private final TalonFX follower;

    // Config
    private final TalonFXConfiguration motorConfig;

    // Status Signals
    StatusSignal<Current> indexerAmpsActive;
    StatusSignal<AngularVelocity> indexerVelocity;
    StatusSignal<Voltage> indexerVoltage;
    StatusSignal<Voltage> indexerFollowerVoltage;

    // Control Requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut dutyCycleRequest2 = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final Follower followerRequest = new Follower(TalonFXConstants.MOTOR_ID, MotorAlignmentValue.Opposed);

    public IndexerIOTalonFX() {
        motor = new TalonFX(TalonFXConstants.MOTOR_ID);
        follower = new TalonFX(TalonFXConstants.FOLLOWER_MOTOR_ID);

        motorConfig = new TalonFXConfiguration()
            .withSlot0(TalonFXConstants.MOTOR_GAINS)
            .withCurrentLimits(TalonFXConstants.MOTOR_CURRENT_LIMITS)
            .withOpenLoopRamps(TalonFXConstants.MOTOR_OPEN_LOOP_RAMPS)
            .withFeedback(TalonFXConstants.MOTOR_FEEDBACK_CONFIGS)
            .withMotorOutput(TalonFXConstants.MOTOR_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
        PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(motorConfig));

        indexerAmpsActive = motor.getStatorCurrent();
        indexerVelocity = motor.getVelocity();
        indexerVoltage = motor.getMotorVoltage();
        indexerFollowerVoltage = follower.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50, indexerAmpsActive, indexerVelocity,
            indexerVoltage);

        follower.setControl(new Follower(TalonFXConstants.MOTOR_ID, MotorAlignmentValue.Opposed));

        motor.optimizeBusUtilization();
        follower.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerMotorConnected = BaseStatusSignal.refreshAll(indexerAmpsActive,
                indexerVelocity, indexerVoltage)
            .isOK();
        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(indexerFollowerVoltage).isOK();

        inputs.indexerVelocityRPS = indexerVelocity.getValue().in(Units.RotationsPerSecond)
            / TalonFXConstants.GEAR_RATIO;

        inputs.indexerCurrentAmps = indexerAmpsActive.getValueAsDouble();

        inputs.indexerVoltage = indexerVoltage.getValueAsDouble();
    }

    @Override
    public void setIndexerVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
        follower.setControl(followerRequest);
    }

    @Override
    public void setIndexerOpenLoop(double input) {
        motor.setControl(dutyCycleRequest.withOutput(input));
        follower.setControl(followerRequest);
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        motor.setControl(velocityRequest
            .withVelocity(Units.RotationsPerSecond.of(
                velocity)));
        follower.setControl(followerRequest);

    }

    @Override
    public void stop() {
        motor.stopMotor();
        follower.stopMotor();
    }

}
