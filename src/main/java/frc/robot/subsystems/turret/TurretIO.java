package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean turnMotorConnected = false;
        public Voltage turnAppliedVolts = Volts.zero();
        public Current turnCurrent = Amps.zero();
        public Angle turnPosition = Radians.zero();
        public AngularVelocity turnVelocity = RadiansPerSecond.zero();

        public double hoodPosition = 0;
        public boolean hoodAtPosition = false;

        public boolean flywheelMotorConnected = false;
        public Voltage flywheelAppliedVolts = Volts.zero();
        public Current flywheelCurrent = Amps.zero();
        public AngularVelocity flywheelSpeed = RadiansPerSecond.zero();
        public AngularAcceleration flywheelAccel = RadiansPerSecond.per(Units.Second).zero();
    }
    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurnSetpoint(Angle position, AngularVelocity velocity) {}

    public default void setHoodPosition(double position) {}

    public default void setFlywheelSpeed(AngularVelocity speed) {}

    public default void stopTurn() {}

    public default void stopHood() {}

    public default void stopFlywheel() {}

    public default void setTurnPIDConstants(double kP, double kD, double kV, double kS) {}

    public default void setFlywheelPIDConstants(double kP, double kD, double kV, double kS) {}
}
