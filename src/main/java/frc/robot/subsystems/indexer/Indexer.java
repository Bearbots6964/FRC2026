package frc.robot.subsystems.indexer;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase{
    private final IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io){
        this.io = io;
    }


    @Override 
    public void periodic(){
        io.updateInputs(inputs);
    }

    public Command runIndexerOpenLoopCommand(double input){
        return Commands.run(() -> io.setIndexerOpenLoop(input * IndexerConstants.MOTOR_SPEED_PERCENTAGE), this);
    }

    public Command runIndexerVelocityCommand(double velocity){
        return Commands.run(() -> io.setIndexerVelocity(velocity), this);

    }

    public Command runIndexerVoltageCommand(double volts){
        return Commands.run(() -> io.setIndexerVoltage(volts), this);
    }
}
