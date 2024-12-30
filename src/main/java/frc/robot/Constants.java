package frc.robot;



import org.photonvision.PhotonCamera;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {

    /*  JUST FYI GEAR RATIO INFORMATION               */
    /*  Drive Motor (Talon FX) Gear Ration  6.75: 1   */
    /*  Turn Motor (Neos) Gear Ratio 21.419 : 1       */

        //Battery Voltage
        public static final double MAX_VOLTAGE = 12.0;


        public static final class DriveConstants {

            //Mk4I_L2 configuraiton
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.82);
            public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
            public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

                //*****************************************************************//
            //*****************************************************************//
            //  OVERALL ROBOT SPEED SCALED DOWN BY THIS PERCENTAGE FOR OUTPUT POWER//

            public static final double TELEOP_INIT_THROTTLE = 0.2; 
            public static final double THROTTLE_MIN = 0.08;
            public static final double THROTTLE_MAX = 1;


            //public static final double kTurnMinOffset = 0.47;
            //public static final double kDriveMinOffset = 0.18;

            //*****************************************************************//
            //*****************************************************************//




            //  PHYSICAL WHEEL POSITIONS
            public static final double TRACK_WIDTH = Units.inchesToMeters(21);
            // Distance between right and left wheels
            public static final double TRACK_LENGTH = Units.inchesToMeters(29);
            // Distance between front and back wheels

            //KINEMATICS KNOWS PHYSICAL WHEEL LOCATIONS FOR TRIG WITH SWREVE DRIVE MATH
            public final static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DriveConstants.TRACK_LENGTH / 2.0, DriveConstants.TRACK_WIDTH / 2.0),
                // Front right
                new Translation2d(DriveConstants.TRACK_LENGTH / 2.0, -DriveConstants.TRACK_WIDTH / 2.0),
                // Back left
                new Translation2d(-DriveConstants.TRACK_LENGTH / 2.0, DriveConstants.TRACK_WIDTH / 2.0),
                // Back right
                new Translation2d(-DriveConstants.TRACK_LENGTH / 2.0, -DriveConstants.TRACK_WIDTH / 2.0)
            );


            //Swerve Moules have a DriveMotor (Talon FX), Turning Motor (Neo), 
            //and AbsoluteEncoder (CTRE) to keep track of wheel turn position
            //DRIVE MOTOR PORT AND ABS ENCODER PORT DEFINED BY PHEONIX TUNER
            //TURNING MOTOR PORT DEFINED BY REV SPARK MAX CLIENT
            public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 11;
            public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 12;
            public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 14;
            public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 13;
        

            public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 21;
            public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 22;
            public static final int BACK_LEFT_TURNING_MOTOR_PORT = 24;
            public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 23;


            public static final int FRONT_LEFT_TURNING_ABS_ENCODER_PORT = 31;
            public static final int FRONT_RIGHT_TURNING_ABS_ENCODER_PORT = 32;
            public static final int BACK_LEFT_TURNING_ABS_ENCODER_PORT = 34;
            public static final int BACK_RIGHT_TURNING_ABS_ENCODER_PORT = 33;


            public static final double FRONT_LEFT_TURNING_OFFSET = -0.543701 * 360;
            public static final double FRONT_RIGHT_TURNING_OFFSET = 0.305664 * 360;
            public static final double BACK_LEFT_TURNING_OFFSET = -0.031006 * 360;
            public static final double BACK_RIGHT_TURNING_OFFSET = 0.302002 * 360;
        
            //eef freef
            //abosolute encoder on swerve modules is not manufactured with a perfect zero position.
            //the offset is measured with the Pheonix Tuner app and recorded here so it can be compensated
            //for when SwerveModules are created in SwerveSubsystem
            public static final double FRONT_LEFT_ABS_ENC_TURNING_OFFSET = FRONT_LEFT_TURNING_OFFSET * Math.PI / 180.0;
            public static final double FRONT_RIGHT_ABS_ENC_TURNING_OFFSET = FRONT_RIGHT_TURNING_OFFSET * Math.PI / 180.0; 
            public static final double BACK_LEFT_ABS_ENC_TURNING_OFFSET = BACK_LEFT_TURNING_OFFSET * Math.PI / 180.0;
            public static final double BACK_RIGHT_ABS_ENC_TURNING_OFFSET = BACK_RIGHT_TURNING_OFFSET * Math.PI / 180.0;

            //ACTUAL MAX PHYSICAL SPEEDS
            public static final double PHYSICAL_MAX_SPEED_MPS = 6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
            public static final double PHYSICAL_MAX_ACC_MPS2 = 3;
            public static final double PHYSICAL_MAX_ANG_SPEED_RPS = PHYSICAL_MAX_SPEED_MPS / Math.hypot(TRACK_WIDTH / 2.0, TRACK_LENGTH / 2.0);
            public static final double PHYSICAL_MAX_ANG_SPEED_RPS2 = 4;

         }



        public static final class AutoConstants {
        

            public static final double THETA_kP = 1.9;
            public static final double THETA_kI = 0.0;
            public static final double THETA_kD = 0.05;
        
        
            public static final double TRANSLATION_kP = 0.5;
            public static final double TRANSLATION_kI = 0.0;
            public static final double TRANSLATION_kD = 0.0;

            
        }





        //Joystick buttons
        public static final class OIConstants {
            //controller number defined by Drive Station 
            public static final int DRIVER_CONTROLLER_PORT = 0;
            public static final int ARM_CONTROLLER_PORT = 1;

            //joystick controller numbers can be found in Drive Station
            public static final int DRIVER_YAXIS = 1;
            public static final int DRIVER_XAXIS = 0;
            public static final int DRIVER_ROTAXIS = 4;
            
     
            public static final int DRIVER_LEFT_SPIN = 2;
            public static final int DRIVER_RIGHT_SPIN = 3;
            public static final int POV_INDEX = 0;
            public static final int ALIGN_TARGET = 10;




            //if Joystick is sticky around zero, deadband keeps it from thinking
            //it sees an input
            public static final double DEADBAND = 0.05;
        }







        public static class VisionConstants {

        
            /// Minimum target ambiguity. Targets with higher ambiguity will be discarded //
            public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

                
            public static final double GAME_PIECE_TARG_DIST = .3;
            public static final double GAME_PIECE_HEIGHT = Units.inchesToMeters(2);
            public static final PhotonCamera CAMERA_GAME_PIECE_LEFT = new PhotonCamera("cameraGamePieceLeft");
            public static final PhotonCamera CAMERA_GAME_PIECE_RIGHT = new PhotonCamera("cameraGamePieceRight");
            public static final PhotonCamera CAMERA_TAG_FRONT = new PhotonCamera("cameraTagFront");
            public static final PhotonCamera CAMERA_TAG_BACK = new PhotonCamera("cameraTagBack");

            public static final PhotonCamera[] CAMERAS_GAME_PIECE = new PhotonCamera[]{
                CAMERA_GAME_PIECE_LEFT, CAMERA_GAME_PIECE_RIGHT
            };

            public static final PhotonCamera[] CAMERAS_TAG = new PhotonCamera[]{
                    CAMERA_TAG_FRONT, CAMERA_TAG_BACK
            };
    

            public static final Transform3d[] CAMERAS_TRANSFORM3DS_TAG = new Transform3d[]{
            // FRONT
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(17), Units.inchesToMeters(0),Units.inchesToMeters(3.5)),
                    new Rotation3d(180, Math.toRadians(0), Math.toRadians(0))),
                //  BACK   
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(-17), Units.inchesToMeters(0),Units.inchesToMeters(3.5)),
                        new Rotation3d(180, Math.toRadians(0), Math.toRadians(180)))
             
            };


            public static final Transform3d[] CAMERAS_TRANSFORM3DS_GAME_PIECE = new Transform3d[]{
                // LEFT
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(13),Units.inchesToMeters(3.5)),
                    new Rotation3d(180, Math.toRadians(45), Math.toRadians(90))),
                //  RIGHT   
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-13),Units.inchesToMeters(3.5)),
                        new Rotation3d(180, Math.toRadians(0), Math.toRadians(-90)))
             
            };
    
 
        }


       
        public static final class CycloidalDriveConstants {
            public static final double GEAR_RATIO = 19 * 7 * 3; //gear ratios - cycloidal, planetary, planetary

            public static final int MOTOR_LEFT_PORT = 43;
            public static final int MOTOR_RIGHT_PORT = 44;


        }
        public static final class IntakeConstants {

            public static final int MOTOR_LEFT_PORT = 41;
            public static final int MOTOR_RIGHT_PORT = 42;


        }


        public static final class FalconMotorConstants {
            public static final int MOTOR_PORT = 44;
            public static final double GEAR_RATIO = 19 * 7 * 3; //gear ratios - cycloidal, planetary, planetary


        }


        public static final class FieldConstants {

            //max shot 20 feet away
            public static final double MAX_SHOT_RANGE = 3.3; //m
            public static final double SHOT_Y_OFFSET = -0.1;

            public static final double FIELD_LENGTH_METERS = 16.45;
            public static final double FIELD_WIDTH_METERS = 8.21;
            // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
            public static final Pose2d FLIPPING_POSE = new Pose2d(
                    new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));



            //tag positions in meters
            // x, y, z, rotx, roty, rotz
            public static final Pose3d[] TAG_POSITIONS = new Pose3d[]{
                new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), //no tag 0 so origin
                new Pose3d(15.08, 0.25, 1.36, new Rotation3d(0, 0, 120.00)), //tag 1
                new Pose3d(16.19, 0.88, 1.36, new Rotation3d(0, 0, 120.00)), //tag 2
                new Pose3d(16.58, 4.98, 1.45, new Rotation3d(0, 0, 180.00)), //tag 3
                new Pose3d(16.58, 5.55, 1.45, new Rotation3d(0, 0, 180.00)), //tag 4
                new Pose3d(14.70, 8.20, 1.36, new Rotation3d(0, 0, -90.00)), //tag 5
                new Pose3d(1.84, 8.20, 1.36, new Rotation3d(0, 0, -90.00)), //tag 6
                new Pose3d(-0.04, 5.55, 1.45, new Rotation3d(0, 0, 0.00)), //tag 7
                new Pose3d(-0.04, 4.98, 1.45, new Rotation3d(0, 0, 0.00)), //tag 8
                new Pose3d(0.36, 0.88, 1.36, new Rotation3d(0, 0, 60.00)), //tag 9
                new Pose3d(1.46, 0.25, 1.36, new Rotation3d(0, 0, 60.00)), //tag 10
                new Pose3d(11.90, 3.71, 1.32, new Rotation3d(0, 0, -60.00)), //tag 11
                new Pose3d(11.90, 4.50, 1.32, new Rotation3d(0, 0, 60.00)), //tag 12
                new Pose3d(11.22, 4.11, 1.32, new Rotation3d(0, 0, 180.00)), //tag 13
                new Pose3d(5.32, 4.11, 1.32, new Rotation3d(0, 0, 0.00)), //tag 14
                new Pose3d(4.64, 4.50, 1.32, new Rotation3d(0, 0, 120.00)), //tag 15
                new Pose3d(4.64, 3.71, 1.32, new Rotation3d(0, 0, -120.00)) //tag 16
                
            };
        

        }

        public static final class LEDConstants {
            public static final int BLINKIN_PWM_PORT = 0;
            /*
            This table primarily describes the behavior of the 5V addressable LEDs as they are capable of more complicated patterns. 12V LEDs will all show patterns with the same color palette as the pattern selected and the speed can be adjusted as indicated for the selected pattern. REV-11-1105-UM-0 Copyright © 2018 REV Robotics, LLC 15
            */
            public static final double RAINBOW1 = -0.99; // Fixed Palette Pattern Rainbow, Rainbow Palette Pattern Density Speed Brightness 
            public static final double RAINBOW2 = -0.97; //Fixed Palette Pattern Rainbow, Party Palette Pattern Density Speed Brightness 
            public static final double RAINBOW3 = -0.95; //Fixed Palette Pattern Rainbow, Ocean Palette Pattern Density Speed Brightness 
            public static final double RAINBOW4 =  -0.93; //Fixed Palette Pattern Rainbow, Lave Palette Pattern Density Speed Brightness 
            public static final double RAINBOW5 =   -0.91; //Fixed Palette Pattern Rainbow, Forest Palette Pattern Density Speed Brightness 
            public static final double RAINBOW6 =  -0.89; //Fixed Palette Pattern Rainbow with Glitter Pattern Density Speed Brightness 
            public static final double CONFETTI =  -0.87; //Fixed Palette Pattern Confetti Pattern Density Speed Brightness 
            public static final double SHOT_RED =  -0.85; //Fixed Palette Pattern Shot, Red - - Brightness
            public static final double SHOT_BLUE=  -0.83; //Fixed Palette Pattern Shot, Blue - - Brightness 
            public static final double SHOT_WHITE =  -0.81; //Fixed Palette Pattern Shot, White - - Brightness 
            public static final double SINELON_RAINBOW =  -0.79; //Fixed Palette Pattern Sinelon, Rainbow Palette Pattern Density Speed Brightness 
            public static final double SINELON_PARTY = -0.77; //Fixed Palette Pattern Sinelon, Party Palette Pattern Density Speed Brightness 
            public static final double SINELON_OCEAN = -0.75; //Fixed Palette Pattern Sinelon, Ocean Palette Pattern Density Speed Brightness 
            public static final double SINELON_LAVA = -0.73; //Fixed Palette Pattern Sinelon, Lava Palette Pattern Density Speed Brightness 
            public static final double SINELON_FOREST = -0.71; //Fixed Palette Pattern Sinelon, Forest Palette Pattern Density Speed Brightness 
            public static final double BPM_RAINBOW = -0.69; //Fixed Palette Pattern Beats per Minute, Rainbow Palette Pattern Density Speed Brightness 
            public static final double BPM_PARTY = -0.67; //Fixed Palette Pattern Beats per Minute, Party Palette Pattern Density Speed Brightness 
            public static final double BPM_OCEAN = -0.65; //Fixed Palette Pattern Beats per Minute, Ocean Palette Pattern Density Speed Brightness 
            public static final double BPM_LAVA =  -0.63; //Fixed Palette Pattern Beats per Minute, Lava Palette Pattern Density Speed Brightness 
            public static final double BPM_FOREST =  -0.61; //Fixed Palette Pattern Beats per Minute, Forest Palette Pattern Density Speed Brightness 
            public static final double BPM_FIRE_MED =  -0.59; //Fixed Palette Pattern Fire, Medium - - Brightness 
            public static final double BPM_FIRE_LARGE =  -0.57; //Fixed Palette Pattern Fire, Large - - Brightness 
            public static final double TWINKLIES_RAINBOW =  -0.55; //Fixed Palette Pattern Twinkles, Rainbow Palette - - Brightness 
            public static final double TWINKLIES_PARTY =  -0.53; //Fixed Palette Pattern Twinkles, Party Palette - - Brightness 
            public static final double TWINKLIES_OCEAN =  -0.51; //Fixed Palette Pattern Twinkles, Ocean Palette - - Brightness 
            public static final double TWINKLIES_LAVA =  -0.49; //Fixed Palette Pattern Twinkles, Lava Palette - - Brightness 
            public static final double TWINKLIES_FOREST =  -0.47; //Fixed Palette Pattern Twinkles, Forest Palette - - Brightness 
            public static final double WAVES_RAINBOW =  -0.45; //Fixed Palette Pattern Color Waves, Rainbow Palette - - Brightness 
            public static final double WAVES_PARTY =  -0.43; //Fixed Palette Pattern Color Waves, Party Palette - - Brightness 
            public static final double WAVES_OCEAN =  -0.41; //Fixed Palette Pattern Color Waves, Ocean Palette - - Brightness 
            public static final double WAVES_LAVA =  -0.39; //Fixed Palette Pattern Color Waves, Lava Palette - - Brightness 
            public static final double WAVES_FOREST =  -0.37; //Fixed Palette Pattern Color Waves, Forest Palette - - Brightness 
            public static final double SCANNER_RED = -0.35; //Fixed Palette Pattern Larson Scanner, Red Pattern Width Speed Brightness 
            public static final double SCANNER_GRAY = -0.33; //Fixed Palette Pattern Larson Scanner, Gray Pattern Width Speed Brightness 
            public static final double CHASE_RED =  -0.31; // Fixed Palette Pattern Light Chase, Red Dimming Speed Brightness 
            public static final double CHASE_BLUE= -0.29; //Fixed Palette Pattern Light Chase, Blue Dimming Speed Brightness 
            public static final double CHASE_GRAY = -0.27; //Fixed Palette Pattern Light Chase, Gray Dimming Speed Brightness 
            public static final double HEARTBEAT_RED = -0.25; //Fixed Palette Pattern Heartbeat, Red - - Brightness 
            public static final double HEARTBEAT_BLUE = -0.23; //Fixed Palette Pattern Heartbeat, Blue - - Brightness 
            public static final double HEARTBEAT_WHITE = -0.21; //Fixed Palette Pattern Heartbeat, White - - Brightness 
            public static final double HEARTBEAT_GRAY = -0.19; //Fixed Palette Pattern Heartbeat, Gray - - Brightness 
            public static final double BREATH_RED = -0.17; //Fixed Palette Pattern Breath, Red - - Brightness 
            public static final double BREATH_BLUE =  -0.15; //Fixed Palette Pattern Breath, Blue - - Brightness 
            public static final double BREATH_GRAY =  -0.13; //Fixed Palette Pattern Breath, Gray - - Brightness 
            public static final double STROBE_RED = -0.11; //Fixed Palette Pattern Strobe, Red - - Brightness 
            public static final double STROBE_BLUE = -0.09; //Fixed Palette Pattern Strobe, Blue - - Brightness 
            public static final double STROBE_GOLD = -0.07; //Fixed Palette Pattern Strobe, Gold - - Brightness 
            public static final double STROBE_WHITE = -0.05; //Fixed Palette Pattern Strobe, White - - Brightness 
            public static final double C1_BLENDTOBLACK =  -0.03; //Color 1 Pattern End to End Blend to Black - - Brightness 
            public static final double C1_SCANNER =  -0.01; //Color 1 Pattern Larson Scanner Pattern Width Speed Brightness 
            public static final double C1_CHASE =  0.01; //Color 1 Pattern Light Chase Dimming Speed Brightness 
            public static final double C1_HEARTBEAT_SLOW =   0.03; //Color 1 Pattern Heartbeat Slow - - Brightness 
            public static final double C1_HEARTBEAT_MED = 0.05; //Color 1 Pattern Heartbeat Medium - - Brightness 
            public static final double C1_HEARTBEAT_FAST = 0.07; //Color 1 Pattern Heartbeat Fast - - Brightness 
            public static final double C1_BREATH_SLOW =  0.09; //Color 1 Pattern Breath Slow - - Brightness 
            public static final double C1_BREATH_FAST =   0.11; //Color 1 Pattern Breath Fast - - Brightness 
            public static final double C1_SHOT =   0.13; //Color 1 Pattern Shot - - Brightness 
            public static final double C1_STROBE =    0.15; //Color 1 Pattern Strobe - - Brightness 
            public static final double C2_BLENDTOBLACK =  0.17; //Color 2 Pattern End to End Blend to Black - - Brightness 
            public static final double C2_SCANNER = 0.19; //Color 2 Pattern Larson Scanner Pattern Width Speed Brightness 
            public static final double C2_CHASE =   0.21; //Color 2 Pattern Light Chase Dimming Speed Brightness 
            public static final double C2_HEARTBEAT_SLOW =   0.23; //Color 2 Pattern Heartbeat Slow - - Brightness 
            public static final double C2_HEARTBEAT_MED = 0.25; //Color 2 Pattern Heartbeat Medium - - Brightness
            public static final double C2_HEARTBEAT_FAST =  0.27; //Color 2 Pattern Heartbeat Fast - - Brightness 
            public static final double C2_BREATH_SLOW =  0.29; //Color 2 Pattern Breath Slow - - Brightness 
            public static final double C2_BREATH_FAST =   0.31; //Color 2 Pattern Breath Fast - - Brightness 
            public static final double C2_SHOT =   0.33; //Color 2 Pattern Shot - - Brightness 
            public static final double C2_STROBE =  0.35; //Color 2 Pattern Strobe - - Brightness
            public static final double C1_C2_SPARKLE =  0.37; //Color 1 and 2 Pattern Sparkle, Color 1 on Color 2 - - Brightness 
            public static final double C2_C1_SPARKLE =  0.39; //Color 1 and 2 Pattern Sparkle, Color 2 on Color 1 - - Brightness 
            public static final double C1_C2_GRADIENT =  0.41; //Color 1 and 2 Pattern Color Gradient, Color 1 and 2 - - Brightness 
            public static final double C1_C2_BPM =  0.43; //Color 1 and 2 Pattern Beats per Minute, Color 1 and 2 Pattern Density Speed Brightness 
            public static final double C1_C2_BLEND1 =   0.45; //Color 1 and 2 Pattern End to End Blend, Color 1 to 2 - - Brightness 
            public static final double C1_C2_BLEND2 =   0.47; //Color 1 and 2 Pattern End to End Blend - - Brightness 
            public static final double C1_C2 = 0.49; //Color 1 and 2 Pattern Color 1 and Color 2 no blending (Setup Pattern) - - Brightness 
            public static final double C1_C2_TWINKLES =  0.51; //Color 1 and 2 Pattern Twinkles, Color 1 and 2 - - Brightness 
            public static final double C1_C2_WAVES =   0.53; //Color 1 and 2 Pattern Color Waves, Color 1 and 2 - - Brightness 
            public static final double C1_C2_SINELON =   0.55; //Color 1 and 2 Pattern Sinelon, Color 1 and 2 Pattern Density Speed Brightness 
            public static final double HOT_PINK = 0.57; //Solid Colors Hot Pink - - Brightness 
            public static final double DARK_RED = 0.59; //Solid Colors Dark red - - Brightness 
            public static final double RED = 0.61; //Solid Colors Red - - Brightness 
            public static final double RED_ORANGE =  0.63; //Solid Colors Red Orange - - Brightness 
            public static final double ORANGE =   0.65; //Solid Colors Orange - - Brightness 
            public static final double GOLD=   0.67; //Solid Colors Gold - - Brightness 
            public static final double YELLOW = 0.69; //Solid Colors Yellow - - Brightness 
            public static final double LAWN_GREEN = 0.71; //Solid Colors Lawn Green - - Brightness 
            public static final double LIME = 0.73; //Solid Colors Lime - - Brightness
            public static final double DARK_GREEN=  0.75; //Solid Colors Dark Green - - Brightness 
            public static final double BLUE_GREEN= 0.79; //Solid Colors Blue Green - - Brightness
            public static final double AQUA=  0.81; //Solid Colors Aqua - - Brightness 
            public static final double SKY_BLUE =  0.83; //Solid Colors Sky Blue - - Brightness 
            public static final double DARK_BLUE =  0.85; //Solid Colors Dark Blue - - Brightness 
            public static final double BLUE =  0.87; //Solid Colors Blue - - Brightness 
            public static final double BLUE_VIOLET =  0.89; //Solid Colors Blue Violet - - Brightness 
            public static final double VIOLET = 0.91; //Solid Colors Violet - - Brightness
            public static final double WHITE =  0.93; //Solid Colors White - - Brightness 
            public static final double GRAY =  0.95; //Solid Colors Gray - - Brightness 
            public static final double DARK_GRAY =  0.97; //Solid Colors Dark Gray - - Brightness 
            public static final double OFF =  0.99; //Solid Colors Black - - Brightness 
        }



}
