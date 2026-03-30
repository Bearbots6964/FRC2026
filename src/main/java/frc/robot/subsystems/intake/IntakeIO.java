package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public boolean intakeConnected = false;
        public boolean intake2Connected = false;

        public boolean intakeDeployConnected = false;

        public Current intakeCurrent = Amps.zero();
        public Voltage intakeVoltage = Volts.zero();
        public AngularVelocity intakeSpeed = RotationsPerSecond.zero();
        public AngularAcceleration intakeAccel = RotationsPerSecondPerSecond.zero();

        public AngularVelocity intakeDeployVelocity = RotationsPerSecond.zero();
        public Current intakeDeployCurrent = Amps.zero();
        public AngularAcceleration intakeDepoloyAcceleration = RotationsPerSecondPerSecond.zero();
        public Angle deployPosition = Rotations.zero();
        public Voltage deployVoltage = Volts.zero();
    }

    public default void stopIntake() {}

    public default void stopDeploy() {}

    public default void runIntakeClosedLoop(AngularVelocity speed) {}

    public default void deployToPosition(Angle position) {}

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setDeployPID(double kP, double kD, double kV, double kS) {}

    public default void setIntakePID(double kP, double kD, double kV, double kS) {}

}