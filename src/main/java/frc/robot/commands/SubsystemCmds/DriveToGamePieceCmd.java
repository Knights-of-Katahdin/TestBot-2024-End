package frc.robot.commands.SubsystemCmds;

import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToGamePieceCmd extends Command {

        private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_SPEED_MPS, DriveConstants.PHYSICAL_MAX_ACC_MPS2);
        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS,DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2);
        
        private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
        private final ProfiledPIDController omegaController = new ProfiledPIDController(10, 0, 0, OMEGA_CONSTRAINTS);


        private final SwerveSubsystem swerveSubsystem;

        private boolean isFinished;

       //private Pose2d bestNotePose;
       private int bestCamera;
       private double bestTargetArea, bestNoteYaw, bestNoteRange, inversionFactor; 

        
        private PhotonPipelineResult[] results = new PhotonPipelineResult[VisionConstants.CAMERAS_GAME_PIECE.length];
        private PhotonTrackedTarget[] targets = new PhotonTrackedTarget[VisionConstants.CAMERAS_GAME_PIECE.length];

        private double[] targetRange = new double[VisionConstants.CAMERAS_GAME_PIECE.length];
        private double[] targetYaw = new double[VisionConstants.CAMERAS_GAME_PIECE.length];
        private double[] targetArea = new double[VisionConstants.CAMERAS_GAME_PIECE.length];
    //    private Translation2d[] translation = new Translation2d[VisionConstants.CAMERAS_NOTE.length];






        public DriveToGamePieceCmd(SwerveSubsystem swerveSubsystem) {

            this.swerveSubsystem = swerveSubsystem;


            yController.setTolerance(0.01);
            omegaController.setTolerance(Units.degreesToRadians(3));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerveSubsystem);
        }

        @Override
        public void initialize() {
            isFinished = false;
            inversionFactor = 1;

            
          //  bestNotePose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)));
            bestCamera = VisionConstants.CAMERAS_GAME_PIECE.length;
            bestTargetArea = 0;
            bestNoteYaw = 0;
            bestNoteRange = 0;




            for(int i = 0; i < VisionConstants.CAMERAS_GAME_PIECE.length; i++){

                List<PhotonPipelineResult> photonPipelineResultList = VisionConstants.CAMERAS_GAME_PIECE[i].getAllUnreadResults();
                if (!photonPipelineResultList.isEmpty()) {
                    // Camera processed a new frame since last
                    // Get the last one in the list.
                    results[i] = photonPipelineResultList.get(photonPipelineResultList.size() - 1);
                    if(results[i].hasTargets()){

                        targets[i] = results[i].getBestTarget();
                        targetRange[i] = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERAS_TRANSFORM3DS_GAME_PIECE[i].getTranslation().getZ(), 
                                                VisionConstants.GAME_PIECE_HEIGHT, 0, Units.degreesToRadians(targets[i].getPitch()));
                        targetYaw[i] = targets[i].getYaw();
                        targetArea[i] = targets[i].getArea();
                //      translation[i] = PhotonUtils.estimateCameraToTargetTranslation(targetRange[i], Rotation2d.fromDegrees(-targetYaw[i]));

                        if(targetArea[i] > bestTargetArea){
                            bestTargetArea = targetArea[i];
                            bestCamera = i;
                        }
                    }
                    if(bestCamera != VisionConstants.CAMERAS_GAME_PIECE.length && (targetRange[bestCamera] > 0 || targetYaw[bestCamera] > 0)){
                //     bestNotePose = new Pose2d(translation[bestCamera].getX(), translation[bestCamera].getY(), new Rotation2d(Math.toRadians(targetYaw[bestCamera])));
                        bestNoteYaw = Math.toRadians(targetYaw[bestCamera]);
                        bestNoteRange = targetRange[bestCamera];
                    }
            
                }
            }

            omegaController.reset(bestNoteYaw);
            yController.reset(bestNoteRange);


        }

        @Override
        public void execute() {
            
            //bestNotePose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)));
            bestCamera = VisionConstants.CAMERAS_GAME_PIECE.length;
            bestTargetArea = 0;


            double ySpeed = 0;
            double omegaSpeed = 0;

            for(int i = 0; i < VisionConstants.CAMERAS_GAME_PIECE.length; i++){

                List<PhotonPipelineResult> photonPipelineResultList = VisionConstants.CAMERAS_GAME_PIECE[i].getAllUnreadResults();
                if (!photonPipelineResultList.isEmpty()) {
                    // Camera processed a new frame since last
                    // Get the last one in the list.
                    results[i] = photonPipelineResultList.get(photonPipelineResultList.size() - 1);
                    if(results[i].hasTargets()){
                        targets[i] = results[i].getBestTarget();
                        targetRange[i] = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERAS_TRANSFORM3DS_GAME_PIECE[i].getTranslation().getZ(), 
                                                VisionConstants.GAME_PIECE_HEIGHT, 0, Units.degreesToRadians(targets[i].getPitch()));
                        targetYaw[i] = targets[i].getYaw();
                        targetArea[i] = targets[i].getArea();
                //      translation[i] = PhotonUtils.estimateCameraToTargetTranslation(targetRange[i], Rotation2d.fromDegrees(-targetYaw[i]));

                        if(targetArea[i] > bestTargetArea){
                            bestTargetArea = targetArea[i];
                            bestCamera = i;
                        }
                    }
                    if(bestCamera != VisionConstants.CAMERAS_GAME_PIECE.length && (targetRange[bestCamera] > 0 || targetYaw[bestCamera] > 0)){
                //     bestNotePose = new Pose2d(translation[bestCamera].getX(), translation[bestCamera].getY(), new Rotation2d(Math.toRadians(targetYaw[bestCamera])));
                        bestNoteYaw = Math.toRadians(targetYaw[bestCamera]);
                        bestNoteRange = targetRange[bestCamera];
                        if(bestCamera % 2 == 1)
                            inversionFactor = 1;
                        else
                            inversionFactor = -1;
                    }

                }
            }
        
            yController.setGoal(0);
            omegaController.setGoal(Math.toRadians(0));

            if (bestTargetArea == 0) {
                // No target has been visible
                swerveSubsystem.stopModules();
                System.out.println("No Game Piece Visible");
                } else {
                    ySpeed = inversionFactor * yController.calculate(bestNoteRange);
                    if (yController.atGoal()) {
                        ySpeed = 0;
                    }
        
                    omegaSpeed =   omegaController.calculate(bestNoteYaw);
                    if (omegaController.atGoal()) {
                        omegaSpeed = 0;
                    }
        
                    if(yController.atGoal() && omegaController.atGoal()){
                        isFinished = true;
                    }
                    
        
                    if(bestCamera == VisionConstants.CAMERAS_GAME_PIECE.length  ||  bestTargetArea <= 0.01 || bestNoteRange > 100){
                        ySpeed = 0;
                        omegaSpeed = 0;
                    }
        
                    swerveSubsystem.setChassisSpeeds(
                        new ChassisSpeeds(0, ySpeed, omegaSpeed));
                }
                
            

            
            if(yController.atGoal() && omegaController.atGoal())
                isFinished = true;
            
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