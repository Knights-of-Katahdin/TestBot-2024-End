package frc.robot.commands.SubsystemCmds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraightCmd extends Command {

        private SwerveSubsystem swerveSubsystem;
        private double timer;
        private double xSpeed, ySpeed, time;
        private boolean isFinished;

        public DriveStraightCmd(SwerveSubsystem swerveSubsystem, double xSpeed, double ySpeed, double time) {
                 this.swerveSubsystem = swerveSubsystem;
                 this.xSpeed = xSpeed;
                 this.ySpeed = ySpeed;
                 this.time = time;
                //must define which subsystems the command needs to operate
                addRequirements(swerveSubsystem);
        }

        @Override
        public void initialize() {
            xSpeed = xSpeed * DriveConstants.PHYSICAL_MAX_SPEED_MPS;
            ySpeed = ySpeed * DriveConstants.PHYSICAL_MAX_SPEED_MPS;
            isFinished = false;
            timer = 0;
            swerveSubsystem.setThrottle(1);
            
        }

        @Override
        public void execute() {

            System.out.println("timer " + timer / 50);
            if(timer/50 >= time){
                isFinished = true;

            }
            else {
                timer++;
                swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(xSpeed, ySpeed, 0.0));
            }
            

        }



        @Override
        public void end(boolean interrupted) {
            swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        }


        @Override
        public boolean isFinished() {
            return isFinished;
        }
    
}
