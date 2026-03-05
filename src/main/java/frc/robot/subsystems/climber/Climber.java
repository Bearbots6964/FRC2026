// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.ClimberConstants.climberMotorConstants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    // will compile after run
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    //Alert for climber disconnection
    private final Alert climberMotorDisconnectedAlert;
    private final Alert climberEncoderDisconnectedAlert;

    //triggers: prevent subsystems from knowing each other to reduce dependency
    public final Trigger isAtTop = new Trigger( //trigger that is true when the climber has fully climbed
        () -> Units.radiansToDegrees(inputs.climberEncoderAbsolutePositionRad) 
              >= climberMotorConstants.climbDegrees);
    
    public final Trigger isAtBottom = new Trigger( //trigger that is true when the climber has fully descended
        () -> Units.radiansToDegrees(inputs.climberEncoderAbsolutePositionRad) 
              <= climberMotorConstants.descendDegrees);

    public final Trigger isClimberMotorDisconnected = new Trigger(
        () -> !inputs.climberMotorConnected);

    public final Trigger isClimberEncoderDisconnected = new Trigger(
        () -> !inputs.climberEncoderConnected);
  
  /** Creates a new climber. */
  public Climber(ClimberIO io) {
    this.io = io;
    climberMotorDisconnectedAlert =
       new Alert("Climber motor disconnected.", Alert.AlertType.kError);
         //critical alert for motor disconnection
    climberEncoderDisconnectedAlert = 
       new Alert("Climber encoder disconnected.", Alert.AlertType.kError);
         //critical alert for encoder disconnection
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    //check for climber motor disconnection and display if true
    climberMotorDisconnectedAlert.set(!inputs.climberMotorConnected);
    //check for climber encoder disconnection and display if true
    climberEncoderDisconnectedAlert.set(!inputs.climberEncoderConnected);
  }

  //command to climb
  public Command climb(){
    return runEnd(
      () -> io.setClimberVoltage(climberMotorConstants.climberVoltage), //climb at 6 volts when this command is scheduled
      () -> io.setClimberVoltage(0.0) //stop the climber motor when the command ends
    ).until(() -> Units.radiansToDegrees(inputs.climberEncoderAbsolutePositionRad) > climberMotorConstants.climbDegrees) //use absolute position from CANcoder
     .withTimeout(climberMotorConstants.climbTimeoutSeconds); //safety timeout in case encoder fails or mechanism stalls
  }
  
  //command to descend
  public Command descend(){
    return runEnd(
      () -> io.setClimberVoltage(-climberMotorConstants.climberVoltage), //descend at 6 volts when this command is scheduled
      () -> io.setClimberVoltage(0.0) //stop the climber motor when the command ends
    ).until(() -> Units.radiansToDegrees(inputs.climberEncoderAbsolutePositionRad) < climberMotorConstants.descendDegrees) //use absolute position from CANcoder
     .withTimeout(climberMotorConstants.climbTimeoutSeconds); //safety timeout in case encoder fails or mechanism stalls
  }

  //stop the climber motor
  public void stopClimber(){io.stopClimber();}


}
