package frc.robot.commands.SubsystemCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CycloidalDriveSubsystem;




public class CycloidalDriveCmd extends Command {
    private CycloidalDriveSubsystem subsystem;
    private double targetPosition, exitCmdTolerance;
    private boolean isFinished, defaultCmd;

    
    public CycloidalDriveCmd(CycloidalDriveSubsystem subsystem) {
        this.subsystem = subsystem;
        this.defaultCmd = true;
        addRequirements(subsystem);
    }




    public CycloidalDriveCmd(CycloidalDriveSubsystem subsystem,  double targetPosition) {
        this.subsystem = subsystem;
        this.defaultCmd = false;
        this.targetPosition = targetPosition;
        addRequirements(subsystem);
    }



    @Override
    public void initialize() {
        

        exitCmdTolerance = 3; //degrees
        this.isFinished = false;
        if(defaultCmd){
            this.targetPosition = subsystem.getTargetPosition();

        }
        subsystem.setPosition(targetPosition);


        
    }

    @Override
    public void execute() {
        if(Math.abs(targetPosition - subsystem.getAvgMeasuredPosition()) < exitCmdTolerance)
             this.isFinished = true;
    }

    @Override
    public void end(boolean interrupted) { 
    
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }
    
  
}

    