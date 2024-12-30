
package frc.robot.subsystems;


//USES FORK LIBRARY DEVELOPED BY democat3457


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.PhotonRunnable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import frc.robot.com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import frc.robot.com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import frc.robot.com.swervedrivespecialties.swervelib.MotorType;
import frc.robot.com.swervedrivespecialties.swervelib.SwerveModule;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;


public class SwerveSubsystem extends SubsystemBase {


        NetworkTable table = NetworkTableInstance.getDefault().getTable("networkTable");

        NetworkTableEntry ROBOTposeX = table.getEntry("ROBOTposeX");
        NetworkTableEntry ROBOTposeY = table.getEntry("ROBOTposeY");
        NetworkTableEntry ROBOTposeTheta = table.getEntry("ROBOTposeTheta");

            // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
            // you trust your various sensors. Smaller numbers will cause the filter to
            // "trust" the estimate from that particular component more than the others. 
            // This in turn means the particualr component will have a stronger influence
            // on the final pose estimate.

            /**
             * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
             * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
             */
            private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
            
            /**
             * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
             * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
             */
            private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
 

      


      //ODOMETRY SETUP
      private final SwerveDrivePoseEstimator poseEstimator;

      private final PhotonRunnable photonCameraFront = new PhotonRunnable(VisionConstants.CAMERAS_TAG[0],  VisionConstants.CAMERAS_TRANSFORM3DS_TAG[0], this::addVisionMeasurement);
      private final PhotonRunnable photonCameraBack = new PhotonRunnable(VisionConstants.CAMERAS_TAG[1], VisionConstants.CAMERAS_TRANSFORM3DS_TAG[1], this::addVisionMeasurement);


     private final Thread photonThreadFront = new Thread(photonCameraFront);
     private final Thread photonThreadBack = new Thread(photonCameraBack);

      private final ReadWriteLock odometryLock = new ReentrantReadWriteLock();
      private OriginPosition originPosition = kBlueAllianceWallRightSide;



  

      //CREATE SWERVE MODULES.  MkSwerveModuleBuilder uses FORK library
      //to configure motors and encoders for SwerveModule 
        public static final MechanicalConfiguration MK4I_L2 = new MechanicalConfiguration(
              DriveConstants.WHEEL_DIAMETER, //wheel diameter
              DriveConstants.DRIVE_REDUCTION,
              false,   
              DriveConstants.STEER_REDUCTION,
              false);




      private SwerveModule frontLeft = new MkSwerveModuleBuilder()
              .withGearRatio(MK4I_L2)
              .withDriveMotor(MotorType.FALCON, DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT)
              .withSteerMotor(MotorType.NEO, DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT)
              .withSteerEncoderPort(DriveConstants.FRONT_LEFT_TURNING_ABS_ENCODER_PORT)
              .withSteerOffset(DriveConstants.FRONT_LEFT_ABS_ENC_TURNING_OFFSET)
              .build();
      
    
      private SwerveModule frontRight = new MkSwerveModuleBuilder()
            .withGearRatio(MK4I_L2)
            .withDriveMotor(MotorType.FALCON, DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT)
            .withSteerMotor(MotorType.NEO, DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT)
            .withSteerEncoderPort(DriveConstants.FRONT_RIGHT_TURNING_ABS_ENCODER_PORT)
            .withSteerOffset(DriveConstants.FRONT_RIGHT_ABS_ENC_TURNING_OFFSET)
            .build();
    
    
    
      private final SwerveModule backLeft = new MkSwerveModuleBuilder()
            .withGearRatio(MK4I_L2)
            .withDriveMotor(MotorType.FALCON, DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT)
            .withSteerMotor(MotorType.NEO, DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT)
            .withSteerEncoderPort(DriveConstants.BACK_LEFT_TURNING_ABS_ENCODER_PORT)
            .withSteerOffset(DriveConstants.BACK_LEFT_ABS_ENC_TURNING_OFFSET)
            .build();
      
    
    
      private final SwerveModule backRight = new MkSwerveModuleBuilder()
              .withGearRatio(MK4I_L2)
              .withDriveMotor(MotorType.FALCON, DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT)
              .withSteerMotor(MotorType.NEO, DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT)
              .withSteerEncoderPort(DriveConstants.BACK_RIGHT_TURNING_ABS_ENCODER_PORT)
              .withSteerOffset(DriveConstants.BACK_RIGHT_ABS_ENC_TURNING_OFFSET)
              .build();


      //states of each swerveModule.  each swerveModule checks its state to determine how it should mov
      public SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.0)),
                new SwerveModuleState(0.0, new Rotation2d(0.0)),
                new SwerveModuleState(0.0, new Rotation2d(0.0)),
                new SwerveModuleState(0.0, new Rotation2d(0.0))};
  

      //overall robot speed control
      private double throttle = DriveConstants.TELEOP_INIT_THROTTLE;
      private double tempThrottle = DriveConstants.TELEOP_INIT_THROTTLE;



      // Kinematics
      SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
  
      //define gyro and location plugged into RoboRio
      public final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
      RobotConfig config;
    
      
  
      //constructor to initialize chassisSpeeds
      //gyro has startup delay, so initialize gyro heading one second after startup 
      //using its own thread (so other functions aren't held up from processing)
      public SwerveSubsystem() {

          new Thread(() -> {
              try {
                  Thread.sleep(1000);
                  zeroHeading();
              } catch (Exception e) {
              }
          }).start();
          



          poseEstimator =  new SwerveDrivePoseEstimator(
              DriveConstants.kDriveKinematics,
              getRotation2d(),
              getModulePositions(),
              new Pose2d(),
              stateStdDevs, visionMeasurementStdDevs);
 
              
            
            try{
                  config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                  // Handle exception as needed
                  e.printStackTrace();
            }

            // Start PhotonVision thread
            photonThreadFront.setName("PhotonVisionFront");
            photonThreadFront.start();
            photonThreadBack.setName("PhotonVisionBack");
            photonThreadBack.start();


            // Configure AutoBuilder last
            AutoBuilder.configure(
                        this::getCurrentPose, // Robot pose supplier
                        this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
                        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                              new PIDConstants(AutoConstants.TRANSLATION_kP, AutoConstants.TRANSLATION_kI, AutoConstants.TRANSLATION_kD), // Translation PID constants
                              new PIDConstants(AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD) // Rotation PID constants
                        ),
                        config, // The robot configuration
                        () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                        },
                        this // Reference to this subsystem to set requirements
            );

      }


      //set overall robot speed control
      public double getThrottle() { return throttle;}


      public void adjThrottle(double adjustmentAmt) { setThrottle(getThrottle() + adjustmentAmt);}
      
      //set overall robot speed control
      public void setThrottle(double throt) { 
            if(throt <= DriveConstants.THROTTLE_MIN)
                  throttle = DriveConstants.THROTTLE_MIN;
            else if(throt >= DriveConstants.THROTTLE_MAX)
                  throttle =  DriveConstants.THROTTLE_MAX;
            else
            throttle = throt;
      }

      
      public void saveControllerThrottle() { 
            tempThrottle = throttle;
      }

      public void restoreControllerThrottle() { 
            throttle = tempThrottle;
      }


      //field relative 
      public ChassisSpeeds  getFieldRelativeSpeeds() {
            ChassisSpeeds chassisSpeeds =  kinematics.toChassisSpeeds(this.states[0], this.states[1], this.states[2], this.states[3]);
            chassisSpeeds.toRobotRelativeSpeeds(getRotation2d());
            return chassisSpeeds;
             
      }

      //robot relative 
      public ChassisSpeeds  getRobotRelativeSpeeds() {
             return kinematics.toChassisSpeeds(this.states[0], this.states[1], this.states[2], this.states[3]);
      }


      public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
             setChassisSpeeds(chassisSpeeds);
        
      }




      //module states are based on math from kinematics and chassisSpeeds.  
      //kinematics does not change so input is chassisSpeeds.
      //chassisSpeeds is variable is applied to module states in periodic function, 
      //which happens once every 50 seconds
      public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
            this.states = kinematics.toSwerveModuleStates(chassisSpeeds);
      }




      //setModulestates with moduleState array input is required for trajectory following
      public SwerveModulePosition[] getModulePositions() {
            return  new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            };    
      }

      //setModulestates with moduleState array input is required for trajectory following
      public void setModuleStates(SwerveModuleState[] moduleStates) {
            this.states = moduleStates;

            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.PHYSICAL_MAX_SPEED_MPS);
            double throt = this.throttle;
              
            if (RobotState.isAutonomous()) throt = 1.0;
            
            frontLeft.set(states[0].speedMetersPerSecond * throt / DriveConstants.PHYSICAL_MAX_SPEED_MPS * Constants.MAX_VOLTAGE, states[0].angle.getRadians());
            frontRight.set(states[1].speedMetersPerSecond * throt / DriveConstants.PHYSICAL_MAX_SPEED_MPS * Constants.MAX_VOLTAGE, states[1].angle.getRadians());
            backLeft.set(states[2].speedMetersPerSecond * throt / DriveConstants.PHYSICAL_MAX_SPEED_MPS * Constants.MAX_VOLTAGE, states[2].angle.getRadians());
            backRight.set(states[3].speedMetersPerSecond * throt / DriveConstants.PHYSICAL_MAX_SPEED_MPS * Constants.MAX_VOLTAGE, states[3].angle.getRadians());
      }


      public void stopModules() {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
            chassisSpeeds.toRobotRelativeSpeeds(getRotation2d());
            setChassisSpeeds(chassisSpeeds);
      }

      /**
     * Set the wheels to an X pattern to plant the robot.
     */
      public void setWheelsToX() {

          setModuleStates(new SwerveModuleState[] {
            // front left
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            // front right
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            // back left
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
            // back right
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
          });
      }

  

      public void setKinematics(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
      }
  
      public void zeroHeading() {
        gyro.reset();
      }

      //gyro angle with math to keep number between 0-360 degrees
      //negated because NavX is cw positive, and swerve drive calcs require ccw positive
      public double getHeading() {
          return -Math.IEEEremainder(gyro.getAngle(), 360);
      }

      public double getChassisPitch(){
        return  gyro.getPitch();

      }


      public double getCumulativeRotation() {
        return -gyro.getAngle();
      }


      //converts degrees to Rotation2d format, which is the format required by chassisSpeeds
      public Rotation2d getRotation2d() {
            return Rotation2d.fromDegrees(getHeading());
      }


      public Pose2d getCurrentPose() {
            odometryLock.readLock().lock();
            try {
                return poseEstimator.getEstimatedPosition();
            } finally {
                odometryLock.readLock().unlock();
            }
      }

   
      // Resets the current pose to the specified pose. This should ONLY be called
      // when the robot's position on the field is known, like at the beginning of
      // a match.
      // @param newPose new pose
      
      public void setCurrentPose(Pose2d newPose) {
              odometryLock.writeLock().lock();
              try {
                  poseEstimator.resetPosition(getRotation2d(), getModulePositions(), newPose);
              } finally {
                  odometryLock.writeLock().unlock();
              }
      }


      //
      // Add a vision measurement. Call this with a pose estimate from an AprilTag.
      // @param pose2d Pose estimate based on the vision target (AprilTag)
      // @param timestamp timestamp when the target was seen
      //
      public void addVisionMeasurement(Pose2d pose2d, double timestamp) {

            var visionPose2d = pose2d;
            if (originPosition != kBlueAllianceWallRightSide) {
                visionPose2d = flipAlliance(visionPose2d);
            }
            odometryLock.writeLock().lock();
            try {
              // Update pose estimator
              if (pose2d != null) {

                  poseEstimator.addVisionMeasurement(visionPose2d, timestamp);
                  
              }
            } finally {
              odometryLock.writeLock().unlock();
            }
      }
  

       // Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
      // alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
       // @param poseToFlip pose to transform to the other alliance
       // @return pose relative to the other alliance's coordinate system
     //
      private Pose2d flipAlliance(Pose2d poseToFlip) {
            return poseToFlip.relativeTo(FieldConstants.FLIPPING_POSE);
      }


      public boolean allianceIsRed(){
        if(DriverStation.getAlliance().isPresent()){
          if(DriverStation.getAlliance().get() == Alliance.Red){
              return true;
          } 
        } 
              return false;
 
      }


 

      @Override
      public void periodic() {
            
             setModuleStates(this.states);
              
            ROBOTposeX.setDouble(getCurrentPose().getX());
            ROBOTposeY.setDouble(getCurrentPose().getY());
            ROBOTposeTheta.setDouble(getCurrentPose().getRotation().getDegrees());
                  
            
           // System.out.println("Current Field Pose " + getCurrentPose());

            odometryLock.writeLock().lock();
            try {
                    poseEstimator.update(getRotation2d(), getModulePositions());    
                    
            } finally {
                    odometryLock.writeLock().unlock();
            }


    }



     
}