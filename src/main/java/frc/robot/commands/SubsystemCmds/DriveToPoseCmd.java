package frc.robot.commands.SubsystemCmds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.FieldConstants.FLIPPING_POSE;

public class DriveToPoseCmd extends Command {

        private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_SPEED_MPS, DriveConstants.PHYSICAL_MAX_ACC_MPS2);
        private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_SPEED_MPS, DriveConstants.PHYSICAL_MAX_ACC_MPS2);
        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS,DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2);
          
        private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
        private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
        private final ProfiledPIDController omegaController = new ProfiledPIDController(5, 0, 0, OMEGA_CONSTRAINTS);

       
        private SwerveSubsystem swerveSubsystem;
        private boolean shouldFlipForAlliance;
        private double x, y, theta;

        private boolean isFinished;



        public DriveToPoseCmd(SwerveSubsystem swerveSubsystem, double x, double y, double theta, boolean shouldFlipForAlliance) {

            this.swerveSubsystem = swerveSubsystem;
            this.shouldFlipForAlliance = shouldFlipForAlliance;
            this.x = x;
            this.y = y;
            this.theta = Math.toRadians(theta);
    
            xController.setTolerance(0.1);
            yController.setTolerance(0.1);
            omegaController.setTolerance(Units.degreesToRadians(2));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerveSubsystem);
        }

        public DriveToPoseCmd(SwerveSubsystem swerveSubsystem, Pose2d pose2d, boolean shouldFlipForAlliance) {

            this.swerveSubsystem = swerveSubsystem;
            this.shouldFlipForAlliance = shouldFlipForAlliance;
            this.x = pose2d.getX();
            this.y = pose2d.getY();
            this.theta = pose2d.getRotation().getRadians();
    
            xController.setTolerance(0.1);
            yController.setTolerance(0.1);
            omegaController.setTolerance(Units.degreesToRadians(2));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerveSubsystem);
        }


        

        @Override
        public void initialize() {
            isFinished = false;

            var robotPose = swerveSubsystem.getCurrentPose();

            omegaController.reset(robotPose.getRotation().getRadians());
            xController.reset(robotPose.getX());
            yController.reset(robotPose.getY());
            


        }

        @Override
        public void execute() {

            var robotPose2d = swerveSubsystem.getCurrentPose();
            Pose2d goalPose2d = new Pose2d(x, y, new Rotation2d(theta));
    
            double xSpeed = 0;
            double ySpeed = 0;
            double omegaSpeed = 0;
            double invertController = 1;

            
            if(shouldFlipForAlliance){
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    goalPose2d = goalPose2d.relativeTo(FLIPPING_POSE);
                     invertController = -1;
                } 
            }
         
            System.out.println("Robot Pose " + robotPose2d);
            System.out.println("Goal Pose " + goalPose2d);


            // Drive
            xController.setGoal(goalPose2d.getX());
            yController.setGoal(goalPose2d.getY());
            omegaController.setGoal(invertController * goalPose2d.getRotation().getRadians());
            


            // Drive to the target
            xSpeed =  invertController * xController.calculate(robotPose2d.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            ySpeed =  invertController * yController.calculate(robotPose2d.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }



            
            
            if(xController.atGoal() && yController.atGoal() && omegaController.atGoal())
                isFinished = true;

            swerveSubsystem.setChassisSpeeds(
                new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
            

            
            
            
        }



        @Override
        public void end(boolean interrupted) {
            swerveSubsystem.stopModules();
        }


        @Override
        public boolean isFinished() {
            return this.isFinished;
        }
        

}