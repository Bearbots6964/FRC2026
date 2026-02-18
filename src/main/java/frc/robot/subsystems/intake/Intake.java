// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
     private final IntakeIO io;
     // will compile after run
      private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    

  /** Creates a new Intake. */
  public Intake(IntakeIO intakeIO) {
    this.io = intakeIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, every 20ms by default
    //update the inputs from the IO and log them
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  //end of periodic
  }
  
  //command to deploy the intake
  public Command deployToPosition(double voltage, double targetPositionRad) {
    //run the deploy motor at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setDeployVoltage(voltage), //deploy at the specified voltage
        () -> io.setDeployVoltage(0.0) //stop the deploy motor when the command ends
    ).until(() -> Math.abs(inputs.deployMotorPositionRad - targetPositionRad) < 0.1); //end the command when the deploy motor is within 0.1 radians of the target position
  }

  public Command retractToPosition(double voltage, double targetPositionRad){
    //run the deploy motor in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setDeployVoltage(-voltage), //retract at the specified voltage
        () -> io.setDeployVoltage(0.0)
    ).until(() -> Math.abs(inputs.deployMotorPositionRad - targetPositionRad) < 0.1); //end the command when the deploy motor is within 0.1 radians of the target position
  }

  //command to intake fuel
  public Command intake(double voltage) {
    //run the intake at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setIntakeVoltage(voltage),
        () -> io.setIntakeVoltage(0.0)
    );
  }

  public Command eject(double voltage) {
    //run the intake in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setIntakeVoltage(-voltage),
        () -> io.setIntakeVoltage(0.0)
    );
  }



//end of class
}
