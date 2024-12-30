
package frc.robot.commands.SubsystemCmds;


import java.util.function.Supplier;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {


    private final SwerveSubsystem swerveSubsystem;


    //Suppliers get inputs from joystick
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, rightSpinFunction, leftSpinFunction;
    private final Supplier<Boolean> alignTarget;
    private final Supplier<Integer> targetTrajectoryFunction;
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS,DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2);
     private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);



    private boolean spinThrottleSaved, targetThrottleSaved, spinThrottleDone, targetThrottleDone, initPressAlignButton, initPressTrajButton, changeKinematics;
    private double tempThrottle, leftTrigger, rightTrigger;

    private boolean spinLatch;
    private double spinSpeed;
    private double SPIN_SPEED = 1;
    private double SPIN_THROTTLE = 1;
    private double currKinematics = 0;
    private boolean shouldFlipForAlliance;
    private double targetRange;



    //slew rate limiter smooths out jerky movements of joystick input
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    //actual x, y, theta speeds of robot chassis
    private ChassisSpeeds chassisSpeeds;


    private SwerveDriveKinematics targetTrajKinematics;

    //Joystick Command Constructor, takes input for use in this class
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
            Supplier<Double> leftSpinFunction, Supplier<Double> rightSpinFunction, Supplier <Boolean> alignTarget, Supplier<Integer> targetTrajectoryFunction,  boolean shouldFlipForAlliance) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.leftSpinFunction = leftSpinFunction;
        this.rightSpinFunction = rightSpinFunction;
        this.alignTarget = alignTarget;
        this.targetTrajectoryFunction = targetTrajectoryFunction;
      
        this.shouldFlipForAlliance = shouldFlipForAlliance;
        this.xLimiter = new SlewRateLimiter(DriveConstants.PHYSICAL_MAX_SPEED_MPS);
        this.yLimiter = new SlewRateLimiter(DriveConstants.PHYSICAL_MAX_SPEED_MPS);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.PHYSICAL_MAX_SPEED_MPS);

        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        

        //must define which subsystems the command needs to operate
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setThrottle(DriveConstants.TELEOP_INIT_THROTTLE);
        spinThrottleSaved = false;

        targetThrottleSaved = false;
        spinThrottleDone = false;
        targetThrottleDone = false;
        initPressAlignButton = true;
        initPressTrajButton = true;
        changeKinematics = false;
        spinLatch = false;
        spinSpeed = SPIN_SPEED;

        

    }

    @Override
    public void execute() {

        swerveSubsystem.setKinematics(DriveConstants.kDriveKinematics);
 
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get()*2;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.DEADBAND ? turningSpeed : 0.0;


        /*********************************************************************************************/
        //Change Kinematics
        leftTrigger = leftSpinFunction.get() > OIConstants.DEADBAND ? leftSpinFunction.get()  : 0.0;
        rightTrigger = rightSpinFunction.get() > OIConstants.DEADBAND ? rightSpinFunction.get() : 0.0;
   
        if(leftTrigger > 0 || rightTrigger > 0){
;
                //store throttle settings before setting input speed
                if(!spinThrottleSaved){
                        tempThrottle = swerveSubsystem.getThrottle();
                        spinThrottleSaved = true;
                }
                //INCREASE SPEED WHEN EXECUTING SPIN
                swerveSubsystem.setThrottle(SPIN_THROTTLE);
                turningSpeed = changeKinematicsSpin(xSpeed, ySpeed);
                xSpeed = 0;
                ySpeed = 0;
                spinThrottleDone = true;
          }else{
                if(spinThrottleDone){
                    swerveSubsystem.setThrottle(tempThrottle);
                    spinThrottleDone = false;
                }
                spinLatch = false;
                currKinematics = 0;
                spinSpeed = SPIN_SPEED;
                spinThrottleSaved = false;

         }
        /*********************************************************************************************/
       
         //Follow Target Trajectory
         if(targetTrajectoryFunction.get() > 0){


            double thetaOffset = 0; //in degrees
            Pose2d targetPose2d = FieldConstants.TAG_POSITIONS[7].toPose2d();
            double flipRobot = Math.toRadians(180); // 180 or 0 degrees
            double turnSpeedTarget = .2;

            if(targetTrajectoryFunction.get() == 90)
                //do nothing
            if(targetTrajectoryFunction.get() == 270)
                thetaOffset = -thetaOffset;

                    
            var robotPose2d = swerveSubsystem.getCurrentPose();



            if(!changeKinematics){

                if(!targetThrottleSaved){
                        tempThrottle = swerveSubsystem.getThrottle();
                        targetThrottleSaved = true;
                    }
                    swerveSubsystem.setThrottle(1);
                    targetThrottleDone = true;
                    if(initPressTrajButton){
                           omegaController.reset(robotPose2d.getRotation().getRadians());      
                    
                    }

                    initPressTrajButton = false;
                    turningSpeed = 0;

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
                        targetRange = Math.hypot((robotPose2d.getY() - targetPose2d.getY()), (robotPose2d.getX() - x));
                        turningSpeed = 0;
                        targetTrajKinematics = changeKinematics(targetRange, thetaOffset);
                        changeKinematics = true;
                    }

            }else if(changeKinematics){
                swerveSubsystem.setKinematics(targetTrajKinematics);
                if(targetTrajectoryFunction.get() == 270)
                    turningSpeed = turnSpeedTarget;
                if(targetTrajectoryFunction.get() == 90)
                    turningSpeed = -turnSpeedTarget;
                
                

            }
        } else {
                initPressTrajButton = true;
                changeKinematics = false;
                
                if(targetThrottleDone){
                    swerveSubsystem.setThrottle(tempThrottle);
                    targetThrottleDone = false;
                }
                targetThrottleSaved = false;
        
        };     
        
        /*********************************************************************************************/
       
        //Align to Target
        if(alignTarget.get()){
                    
                    double thetaOffset = 0; //in degrees
                    double flipRobot = Math.toRadians(180); // 180 or 0 degrees
                    Pose2d targetPose2d = FieldConstants.TAG_POSITIONS[7].toPose2d();


                    var robotPose2d = swerveSubsystem.getCurrentPose();
                   
                    if(initPressAlignButton){
                        omegaController.reset(robotPose2d.getRotation().getRadians());     

                    }
                    initPressAlignButton = false;
                    turningSpeed = 0;

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
    
            
         } else{ 
                
                initPressAlignButton = true;
        };     
        
        /*********************************************************************************************/



        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.PHYSICAL_MAX_SPEED_MPS;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.PHYSICAL_MAX_SPEED_MPS;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2;
    
    
    


        // Relative to field for field oriented driving
        

         chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
         chassisSpeeds.toRobotRelativeSpeeds(swerveSubsystem.getRotation2d());
         swerveSubsystem.setChassisSpeeds(chassisSpeeds);




        //Output chassis speeds to wheels normal kinematics
        swerveSubsystem.setChassisSpeeds(chassisSpeeds);


             


    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

/*********************************************************************************************/    

private SwerveDriveKinematics changeKinematics(double range, double thetaOffset){


        double invertx = 1;
        if(shouldFlipForAlliance){
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                invertx = -1;
            }
        }


        double r_mech = Math.hypot(DriveConstants.TRACK_WIDTH/2, DriveConstants.TRACK_LENGTH/2);
        double theta_mech = Math.toDegrees(Math.atan((DriveConstants.TRACK_WIDTH / 2) / (DriveConstants.TRACK_LENGTH / 2)));

        double flx = (- range  + invertx * r_mech * Math.cos(Math.toRadians(theta_mech - thetaOffset)));
        double fly = (r_mech * Math.sin(Math.toRadians(theta_mech - thetaOffset)));

        double frx = (- range  + invertx * r_mech * Math.cos(Math.toRadians(theta_mech + thetaOffset)));
        double fry = (-r_mech * Math.sin(Math.toRadians(theta_mech + thetaOffset)));

        double blx = (- range  - invertx * r_mech * Math.cos(Math.toRadians(theta_mech + thetaOffset)));
        double bly = (r_mech * Math.sin(Math.toRadians(theta_mech + thetaOffset)));

        double brx = (- range  - invertx * r_mech * Math.cos(Math.toRadians(theta_mech - thetaOffset)));
        double bry = (-  r_mech * Math.sin(Math.toRadians(theta_mech - thetaOffset)));

 
        System.out.println("Target Range: " + range);
        System.out.println("Mechanical Theta: " + theta_mech);
        System.out.println("Offset Theta:  " + thetaOffset);
        System.out.println("Mechanical R: " + r_mech);
        System.out.println("Mechanical Theta: " + theta_mech);
        System.out.println("FL X: " + flx + "FL Y: " + fly);
        System.out.println("FR X: " + frx + "FR Y: " + fry);
        System.out.println("BL X: " + blx + "BL Y: " + bly);
        System.out.println("BR X: " + brx + "BR Y: " + bry);

        return new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(flx, fly),
                    // Front right
                    new Translation2d(frx,  fry),
                    // Back left
                    new Translation2d(blx, bly),
                    // Back right            
                    new Translation2d(brx,  bry)
                );
            
 
    }



   
/*********************************************************************************************/


    private double changeKinematicsSpin(double xSpeed, double ySpeed){
        double currHeading = swerveSubsystem.getHeading();

        double SPEED_TRIGGER_THRESHOLD = 0.45;

        //SET A ROTATION POINT OFF ONE OF THE ROBOT'S WHEELS
        double ROTATION_MAX_OFFSET = 12; //INCHES
        double rotation_offset = ROTATION_MAX_OFFSET;
        if(rightTrigger > 0)
                rotation_offset = Units.inchesToMeters(ROTATION_MAX_OFFSET* (1 - rightTrigger));
        else if(leftTrigger > 0)
                rotation_offset = Units.inchesToMeters(ROTATION_MAX_OFFSET * (1 - leftTrigger));

        double FL = 1;
        double FR = 2;
        double BL = 3;
        double BR = 4;
   
        if(!spinLatch){
            spinSpeed = SPIN_SPEED; 
            //ROBOT FACING 0 DEG HEADING
            if(Math.abs(currHeading) < 45){
                    if(xSpeed < -SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = BR;
                            spinSpeed =  -spinSpeed;           
                        //   System.out.println("Section A");
                        } else if(leftTrigger > 0){
                            currKinematics = BL;
                        //    System.out.println("Section B");
                        }
                        
                    } else if(xSpeed > SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = FR;
                        //    System.out.println("Section C");
                        } else if(leftTrigger > 0){
                            currKinematics = FL;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section D");
                        }
                        
                    }else if(ySpeed < -SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = BR;
                        //    System.out.println("Section E");
                        } else if(leftTrigger > 0){
                            currKinematics = FR;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section F");
                        }
                   
                    } else if(ySpeed > SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = FL;
                        //    System.out.println("Section G");
                        } else if(leftTrigger > 0){
                            currKinematics = BL;
                            spinSpeed = - spinSpeed;
                        //    System.out.println("Section H");
                        }
                   
                    }
            
                //ROBOT FACING 90 DEG HEADING
                }else if(currHeading > 45 && currHeading < 135){
                    if(xSpeed < -SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = BL;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section I");
                        } else if(leftTrigger > 0){
                            currKinematics = FL;
                        //    System.out.println("Section J");
                        }
                    } else if(xSpeed > SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = BR;
                        //    System.out.println("Section K");
                        } else if(leftTrigger > 0){
                            currKinematics = FR;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section L");
                        }
                    }else if(ySpeed < -SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = BL;
                       //     System.out.println("Section M");
                        } else if(leftTrigger > 0){
                            currKinematics = BR;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section N");
                        }
                    } else if(ySpeed > SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = FR;
                        //    System.out.println("Section O");
                        } else if(leftTrigger > 0){
                            currKinematics = FL;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section P");
                        }
                    }
                    
                //ROBOT FACING -90 DEG HEADING
                }else if(currHeading < -45 && currHeading > -135 ){
                    if(xSpeed < -SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = FR;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section Q");
                        } else if(leftTrigger > 0){
                            currKinematics = BR;
                        //    System.out.println("Section R");
                        }
                    } else if(xSpeed > SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = FL;
                        //    System.out.println("Section S");
                        } else if(leftTrigger > 0){
                            currKinematics = BL;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section T");
                        }
                    }else if(ySpeed < -SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = FR;
                        //    System.out.println("Section U");
                        } else if(leftTrigger > 0){
                            currKinematics = FL;
                            spinSpeed =  -spinSpeed;
                        //    System.out.println("Section V");
                        }
                    } else if(ySpeed > SPEED_TRIGGER_THRESHOLD){
                        if(rightTrigger > 0){
                            currKinematics = BL;
                        //    System.out.println("Section W");
                        } else if(leftTrigger > 0){
                            currKinematics = BR;
                            spinSpeed =  -spinSpeed;
                        //   System.out.println("Section X");
                        }
                    }
                    //ROBOT FACING +/- 180 DEG HEADING
                    } else if(Math.abs(currHeading) > 135){
                        if(xSpeed < - SPEED_TRIGGER_THRESHOLD){
                            if(rightTrigger > 0){
                                currKinematics = FL;
                                spinSpeed =  -spinSpeed;
                            //    System.out.println("Section Y");
                            } else if(leftTrigger > 0){
                                currKinematics = FR;
                            //    System.out.println("Section Z");
                            }
                        } else if(xSpeed > SPEED_TRIGGER_THRESHOLD){
                            if(rightTrigger > 0){
                                currKinematics = BL;
                            //    System.out.println("Section AA");
                            } else if(leftTrigger > 0){
                                currKinematics = BR;
                                spinSpeed =  -spinSpeed;
                            //    System.out.println("Section BB");
                            }
                        }else if(ySpeed < -SPEED_TRIGGER_THRESHOLD){

                            if(rightTrigger > 0){
                                currKinematics = FL;
                            //    System.out.println("Section CC");

                            } else if(leftTrigger > 0){
                                currKinematics = BL;
                                spinSpeed =  -spinSpeed;
                            //    System.out.println("Section DD");
                            }

                        } else if(ySpeed > SPEED_TRIGGER_THRESHOLD){
                            if(rightTrigger > 0){
                                currKinematics = FR;
                            //    System.out.println("Section EE");
                                spinSpeed =  -spinSpeed;
                            } else if(leftTrigger > 0){
                                currKinematics = BR;
                            //    System.out.println("Section FF");
                                
                            }
                        
                        } 
                    }
                spinLatch = true;
               // System.out.println("Current Kinnematics: " + currKinematics);
            }



            if(currKinematics == 1)
            {
                swerveSubsystem.setKinematics(
                    new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(-rotation_offset, -rotation_offset),
                        // Front right
                        new Translation2d(-rotation_offset, -DriveConstants.TRACK_WIDTH - rotation_offset),
                        // Back left
                        new Translation2d(-DriveConstants.TRACK_LENGTH - rotation_offset, -rotation_offset),
                        // Back right            
                        new Translation2d(-DriveConstants.TRACK_LENGTH - rotation_offset, -DriveConstants.TRACK_WIDTH - rotation_offset)
                    )
                );
            } else if(currKinematics == 2){
                swerveSubsystem.setKinematics(
                    new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(-rotation_offset, DriveConstants.TRACK_WIDTH + rotation_offset),
                        // Front right
                        new Translation2d(-rotation_offset, rotation_offset),
                        // Back left
                        new Translation2d(-DriveConstants.TRACK_LENGTH  - rotation_offset, DriveConstants.TRACK_WIDTH + rotation_offset),
                        // Back right
                        new Translation2d(-DriveConstants.TRACK_LENGTH - rotation_offset, rotation_offset)                         
                ));

            } else if(currKinematics == 3){
                swerveSubsystem.setKinematics(
                    new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DriveConstants.TRACK_LENGTH + rotation_offset, -rotation_offset),
                        // Front right
                        new Translation2d(DriveConstants.TRACK_LENGTH + rotation_offset, -DriveConstants.TRACK_WIDTH - rotation_offset),
                        // Back left
                        new Translation2d(rotation_offset, -rotation_offset),
                        // Back right
                        new Translation2d(rotation_offset, -DriveConstants.TRACK_WIDTH - rotation_offset)
                ));
            } else if(currKinematics == 4){
                swerveSubsystem.setKinematics(
                    new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DriveConstants.TRACK_LENGTH + rotation_offset, DriveConstants.TRACK_WIDTH + rotation_offset),
                        // Front right
                        new Translation2d(DriveConstants.TRACK_LENGTH + rotation_offset, rotation_offset),
                        // Back left
                        new Translation2d(rotation_offset, DriveConstants.TRACK_WIDTH + rotation_offset),
                        // Back right
                        new Translation2d(rotation_offset, rotation_offset)                        ));
            } else {
                swerveSubsystem.setKinematics(DriveConstants.kDriveKinematics);
            }
            //System.out.println("rotation offset " + rotation_offset);
            return spinSpeed;





    
               

       
    }



}
