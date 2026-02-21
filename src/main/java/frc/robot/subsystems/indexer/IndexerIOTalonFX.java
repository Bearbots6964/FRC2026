package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {
    // Motor
    private final TalonFX motor;

    // Config
    private final TalonFXConfiguration motorConfig;

    // Status Signals
    StatusSignal<Current> indexerAmpsActive;
    StatusSignal<AngularVelocity> indexerVelocity;
    StatusSignal<Voltage> indexerVoltage;

    // Control Requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public IndexerIOTalonFX() {
        motor = new TalonFX(IndexerConstants.TalonFXConstants.MOTOR_ID);

        motorConfig = new TalonFXConfiguration()
                .withSlot0(IndexerConstants.TalonFXConstants.MOTOR_GAINS)
                .withCurrentLimits(IndexerConstants.TalonFXConstants.MOTOR_CURRENT_LIMITS)
                .withMotorOutput(IndexerConstants.TalonFXConstants.MOTOR_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));

        indexerAmpsActive = motor.getStatorCurrent();
        indexerVelocity = motor.getVelocity();
        indexerVoltage = motor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50, indexerAmpsActive, indexerVelocity, indexerVoltage);

        motor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerMotorConnected = BaseStatusSignal.refreshAll(indexerAmpsActive, indexerVelocity, indexerVoltage)
                .isOK();

        inputs.indexerVelocityRPS = indexerVelocity.getValue().in(Units.RotationsPerSecond)
                / IndexerConstants.TalonFXConstants.GEAR_RATIO;

        inputs.indexerCurrentAmps = indexerAmpsActive.getValueAsDouble();

        inputs.indexerVoltage = indexerVoltage.getValueAsDouble();
    }

    @Override
    public void setIndexerVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setIndexerOpenLoop(double input) {
        motor.setControl(dutyCycleRequest.withOutput(input));
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        motor.setControl(velocityRequest
                .withVelocity(Units.RotationsPerSecond.of(velocity * IndexerConstants.TalonFXConstants.GEAR_RATIO)));

    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }

}
