package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.HOOD_SERVO_CHANNEL;
import static frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.HOOD_SERVO_FOLLOWER_CHANNEL;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.TurnMotorConstants;
import frc.robot.util.LinearServo;
import frc.robot.util.PhoenixUtil;
import java.util.prefs.PreferencesFactory;

public class TurretIOTalonFX implements TurretIO {

    private final TalonFX turnMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelFollowerMotor;

    private final CANcoder encoder;

    private final LinearServo hoodServo;
    private final LinearServo hoodFollowerServo;

    private final TalonFXConfiguration turnConfig;
    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration flywheelFollowerConfig;

    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    private final StatusSignal<AngularVelocity> flywheelSpeed;
    private final StatusSignal<AngularAcceleration> flywheelAccel;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Voltage> flywheelFollowerAppliedVolts;

    private final PositionVoltage turnPositionRequest = new PositionVoltage(0);
    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(
        0);
    private final TorqueCurrentFOC flywheelTorqueRequest = new TorqueCurrentFOC(0);

    private final Follower followRequest = new Follower(
        FlywheelMotorConstants.FLYWHEEL_MOTOR_ID,
        FlywheelMotorConstants.FLYWHEEL_OUTPUT_CONFIGS.Inverted
            == FlywheelMotorConstants.FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS.Inverted
            ? MotorAlignmentValue.Aligned
            : MotorAlignmentValue.Opposed
    );

    private final NeutralOut neutralOut = new NeutralOut();

    public TurretIOTalonFX() {
        turnMotor = new TalonFX(TurnMotorConstants.TURN_MOTOR_ID);
        flywheelMotor = new TalonFX(FlywheelMotorConstants.FLYWHEEL_MOTOR_ID);
        flywheelFollowerMotor = new TalonFX(
            FlywheelMotorConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
        encoder = new CANcoder(TurnMotorConstants.TURN_ENCODER_ID);

        hoodServo = new LinearServo(HOOD_SERVO_CHANNEL, 100, 32);
        hoodFollowerServo = new LinearServo(HOOD_SERVO_FOLLOWER_CHANNEL, 100, 32);

        turnConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(TurnMotorConstants.TURN_ENCODER_ID)
                .withSensorToMechanismRatio(TurnMotorConstants.ENCODER_TO_TURRET_RATIO)
                .withRotorToSensorRatio(TurnMotorConstants.MOTOR_TO_TURRET_RATIO
                    / TurnMotorConstants.ENCODER_TO_TURRET_RATIO)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder))
            .withSlot0(TurnMotorConstants.TURN_GAINS)
            .withCurrentLimits(TurnMotorConstants.TURN_CURRENT_LIMITS)
            .withMotorOutput(TurnMotorConstants.TURN_OUTPUT_CONFIGS)
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(TurnMotorConstants.MAX_TURN_ANGLE)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(TurnMotorConstants.MIN_TURN_ANGLE));

        flywheelConfig = new TalonFXConfiguration()
            .withSlot0(FlywheelMotorConstants.FLYWHEEL_GAINS)
            .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
            .withMotorOutput(FlywheelMotorConstants.FLYWHEEL_OUTPUT_CONFIGS)
            .withFeedback(FlywheelMotorConstants.FLYWHEEL_FEEDBACK_CONFIGS);

        flywheelFollowerConfig = new TalonFXConfiguration()
            .withCurrentLimits(FlywheelMotorConstants.FLYWHEEL_CURRENT_LIMITS)
            .withMotorOutput(FlywheelMotorConstants.FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig));
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig));
        PhoenixUtil.tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig));

        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnCurrent = turnMotor.getStatorCurrent();

        flywheelSpeed = flywheelMotor.getVelocity();
        flywheelAccel = flywheelMotor.getAcceleration();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getStatorCurrent();
        flywheelFollowerAppliedVolts = flywheelFollowerMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent,
            flywheelSpeed,
            flywheelAccel,
            flywheelAppliedVolts,
            flywheelCurrent);
        BaseStatusSignal.setUpdateFrequencyForAll(4, flywheelFollowerAppliedVolts);
        turnMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
        flywheelFollowerMotor.optimizeBusUtilization();

        flywheelFollowerMotor.setControl(followRequest);

    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        hoodServo.updateCurPos();
        hoodFollowerServo.updateCurPos();

        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
                turnPosition, turnVelocity, turnAppliedVolts, turnCurrent)
            .isOK();
        inputs.turnPosition = turnPosition.getValue();
        inputs.turnVelocity = turnVelocity.getValue();
        inputs.turnAppliedVolts = turnAppliedVolts.getValue();
        inputs.turnCurrent = turnCurrent.getValue();

        inputs.flywheelMotorConnected = BaseStatusSignal.refreshAll(
                flywheelSpeed, flywheelAccel, flywheelAppliedVolts, flywheelCurrent)
            .isOK();
        inputs.flywheelFollowerMotorConnected = BaseStatusSignal.refreshAll(flywheelFollowerAppliedVolts).isOK();
        inputs.flywheelSpeed = flywheelSpeed.getValue();
        inputs.flywheelAccel = flywheelAccel.getValue();
        inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();

        inputs.hoodPosition = hoodServo.getPosition();
        inputs.hoodPosition = hoodServo.getPosition();
    }

    @Override
    public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
        turnMotor.setControl(turnPositionRequest.withPosition(position).withVelocity(velocity));
    }

    @Override
    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
        hoodFollowerServo.setPosition(position);
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity speed) {
        flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(speed));
    }

    @Override
    public void stopTurn() {
        turnMotor.setControl(neutralOut);
    }

    @Override
    public void stopHood() {
        var position = hoodServo.getPosition();
        hoodServo.set(position);
        hoodFollowerServo.set(position);
    }

    @Override
    public void stopFlywheel() {
        flywheelMotor.setControl(neutralOut);
    }

    @Override
    public void setTurnPIDConstants(double kP, double kD, double kV, double kS) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kD = kD;
        turnConfig.Slot0.kV = kV;
        turnConfig.Slot0.kS = kS;
        PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig));
    }

    @Override
    public void setFlywheelPIDConstants(double kP, double kD, double kV, double kS) {
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kD = kD;
        flywheelConfig.Slot0.kV = kV;
        flywheelConfig.Slot0.kS = kS;
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig));
    }
}
