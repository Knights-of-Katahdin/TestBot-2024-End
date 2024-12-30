package frc.robot.commands.SubsystemCmds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToPoseCmd extends Command {

        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS,DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2);      
        private final ProfiledPIDController omegaController = new ProfiledPIDController(10, 0, 0, OMEGA_CONSTRAINTS);

       
        private SwerveSubsystem swerveSubsystem;
        private Pose2d targetPose2d;
        private double thetaOffset;
        private boolean shouldFlipForAlliance, isFinished;



        public TurnToPoseCmd(SwerveSubsystem swerveSubsystem, double x, double y, double thetaOffset) {

            this.swerveSubsystem = swerveSubsystem;
            this.targetPose2d = new Pose2d(x, y, new Rotation2d(0));
            this.thetaOffset = thetaOffset;
    

            omegaController.setTolerance(Units.degreesToRadians(2));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerveSubsystem);
        }


        public TurnToPoseCmd(SwerveSubsystem swerveSubsystem, Pose2d targetPose2d, double thetaOffset, boolean shouldFlipForAlliance) {

            this.swerveSubsystem = swerveSubsystem;
            this.shouldFlipForAlliance = shouldFlipForAlliance;

            this.targetPose2d = targetPose2d;
            this.thetaOffset = thetaOffset;
    

            omegaController.setTolerance(Units.degreesToRadians(2));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerveSubsystem);
        }

        

        @Override
        public void initialize() {
            isFinished = false;

            var robotPose = swerveSubsystem.getCurrentPose();
            omegaController.reset(robotPose.getRotation().getRadians());

        }

        @Override
        public void execute() {

            double flipRobot = Math.toRadians(180); // 180 or 0 degrees

            var robotPose2d = swerveSubsystem.getCurrentPose();
           
            double turningSpeed = 0;

            double x = targetPose2d.getX();
            thetaOffset = Math.toRadians(thetaOffset);
            if(shouldFlipForAlliance){
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    x = FieldConstants.FIELD_LENGTH_METERS - targetPose2d.getX();
                } 
            } 
            double goalTheta = Math.atan2((robotPose2d.getY() - targetPose2d.getY()), (robotPose2d.getX() - x));
            Pose2d goalPose2d = new Pose2d(0, 0, new Rotation2d(goalTheta));
            Transform2d rotTransform = new Transform2d(0, 0, new Rotation2d(thetaOffset + flipRobot));
            goalPose2d = goalPose2d.transformBy(rotTransform);

            // Drive
            omegaController.setGoal(goalPose2d.getRotation().getRadians());
            turningSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            turningSpeed = turningSpeed / swerveSubsystem.getThrottle();

            if (omegaController.atGoal()) {
                turningSpeed = 0;
            }


            
            if(omegaController.atGoal()){
                  isFinished = true;
            }
            ChassisSpeeds chassisSpeeds =  new ChassisSpeeds(0, 0, turningSpeed);
            chassisSpeeds.toRobotRelativeSpeeds(swerveSubsystem.getRotation2d());
            swerveSubsystem.setChassisSpeeds(chassisSpeeds);
            
            
            
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