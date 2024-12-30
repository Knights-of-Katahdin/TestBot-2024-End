package frc.robot;


import static frc.robot.Constants.FieldConstants.TAG_POSITIONS;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SubsystemCmds.DriveToTagCmd;
import frc.robot.commands.SubsystemCmds.KeepLightCommand;
import frc.robot.commands.SubsystemCmds.CycloidalDriveCmd;
import frc.robot.commands.SubsystemCmds.DriveStraightCmd;
import frc.robot.commands.SubsystemCmds.DriveToGamePieceCmd;
import frc.robot.commands.SubsystemCmds.DriveToPoseCmd;
import frc.robot.commands.SubsystemCmds.SwerveJoystickCmd;
import frc.robot.commands.SubsystemCmds.TestFalconCmd;
import frc.robot.commands.SubsystemCmds.TurnToPoseCmd;
import frc.robot.commands.SubsystemCmds.TurnToTagCmd;
import frc.robot.subsystems.CycloidalDriveSubsystem;
import frc.robot.subsystems.FalconMotorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;




public class RobotContainer {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("networkTable");

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //    private final CycloidalDriveSubsystem cycloidalDriveSubsystem = new CycloidalDriveSubsystem();
    //    private final FalconMotorSubsystem falconMotorTestSubsystem = new FalconMotorSubsystem();
        private final LEDSubsystem ledSubsystem = new LEDSubsystem();



        private final Joystick driverJoystick = new Joystick(OIConstants.DRIVER_CONTROLLER_PORT);
        private final Joystick armJoystick = new Joystick(OIConstants.ARM_CONTROLLER_PORT);


        public static SendableChooser<Command> autoChooser = new SendableChooser<>();
 
        //percent throttle is adjusted per click
        private double THROT_ADJ = 0.08;

        public RobotContainer() {

                // every subsystem need a default command
               swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> -driverJoystick.getRawAxis(OIConstants.DRIVER_YAXIS),
                        () -> driverJoystick.getRawAxis(OIConstants.DRIVER_XAXIS),
                        () -> driverJoystick.getRawAxis(OIConstants.DRIVER_ROTAXIS),
                        () -> driverJoystick.getRawAxis(OIConstants.DRIVER_LEFT_SPIN),
                        () -> driverJoystick.getRawAxis(OIConstants.DRIVER_RIGHT_SPIN),
                        ()-> driverJoystick.getRawButton(OIConstants.ALIGN_TARGET),
                        () -> driverJoystick.getPOV(OIConstants.POV_INDEX),
                        true));


        //        cycloidalDriveSubsystem.setDefaultCommand(new CycloidalDriveCmd(cycloidalDriveSubsystem));
       //         falconMotorTestSubsystem.setDefaultCommand(new TestFalconCmd(falconMotorTestSubsystem));
                ledSubsystem.setDefaultCommand(new KeepLightCommand(ledSubsystem));

                
            //    NamedCommands.registerCommand("ZeroHeadingCmd", Commands.runOnce(swerveSubsystem::zeroHeading, swerveSubsystem));
            NamedCommands.registerCommand("blinky blinky", new InstantCommand(()-> ledSubsystem.setPower(LEDConstants.BPM_RAINBOW), ledSubsystem));
            NamedCommands.registerCommand("go", new InstantCommand(()-> swerveSubsystem.setCurrentPose(new Pose2d(2.930,5.563,new Rotation2d(57.6))), swerveSubsystem));

         
                Shuffleboard.getTab("AUTO").add("Auto Chooser", autoChooser).withSize(3, 2).withPosition(2, 0);
             /////////////   autoChooser.addOption("PathSpeaker1", new PathPlannerAuto("PathSpeaker1"));
                autoChooser.setDefaultOption("go", new PathPlannerAuto("artificial intelligence"));
                autoChooser.addOption("dont go", new PathPlannerAuto("artificial intelligence 2"));

                configureButtonBindings();
 
        }





        // no buttons for this code
        private void configureButtonBindings() {
                 // Button 8 is START
                // Reset zero position of field for field oriented driving
                 new JoystickButton(driverJoystick, 8).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

 
                // Button 5 HELD DOWN is Swerve SLOW MODE
                new JoystickButton(driverJoystick, 5).onTrue(new InstantCommand(()-> swerveSubsystem.saveControllerThrottle()));
                new JoystickButton(driverJoystick, 5).whileTrue(new InstantCommand(()-> swerveSubsystem.setThrottle(DriveConstants.THROTTLE_MIN)));
                new JoystickButton(driverJoystick, 5).onFalse(new InstantCommand(()-> swerveSubsystem.restoreControllerThrottle()));
       
                
                // Button 6 HELD DOWN is Swerve FAST MODE
                new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand(()-> swerveSubsystem.saveControllerThrottle()));
                new JoystickButton(driverJoystick, 6).whileTrue(new InstantCommand(()-> swerveSubsystem.setThrottle(DriveConstants.THROTTLE_MAX)));
                new JoystickButton(driverJoystick, 6).onFalse(new InstantCommand(()-> swerveSubsystem.restoreControllerThrottle()));


                // Y Button THROTTLE UP
               new JoystickButton(driverJoystick, 4).onTrue(new InstantCommand(()-> swerveSubsystem.adjThrottle(THROT_ADJ)));


                // A Button THROTTLE DOWN
                new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(()-> swerveSubsystem.adjThrottle(- THROT_ADJ)));

/* 

               
                new JoystickButton(driverJoystick, 2).whileTrue( 
                        new SequentialCommandGroup( 
                                Commands.runOnce(()->swerveSubsystem.saveControllerThrottle()),
                                Commands.runOnce(()->swerveSubsystem.setThrottle(1)),
                                new DriveToNoteCmd(swerveSubsystem),
                                Commands.runOnce(()->swerveSubsystem.restoreControllerThrottle())));
                
    
                new JoystickButton(driverJoystick, 3).whileTrue( 
                        new SequentialCommandGroup( 
                                Commands.runOnce(()->swerveSubsystem.saveControllerThrottle()),
                                Commands.runOnce(()->swerveSubsystem.setThrottle(1)),
                                new DriveToPoseCmd(swerveSubsystem, 3,5,0),
                                Commands.runOnce(()->swerveSubsystem.restoreControllerThrottle())));
   */             

                //*********** OPERATOR CONTROLLER **********//
                // A Button 
        //        new JoystickButton(armJoystick, 2).onTrue(Commands.runOnce(()->falconMotorTestSubsystem.resetPosition()));
        //        new JoystickButton(armJoystick, 4).onTrue(new TestFalconCmd(falconMotorTestSubsystem, 0));
        //        new JoystickButton(armJoystick, 3).onTrue(new TestFalconCmd(falconMotorTestSubsystem, -80));
        //       new JoystickButton(armJoystick, 1).onTrue(new TestFalconCmd(falconMotorTestSubsystem, 80));





            // DRIVE STRAIGHT COMMAND   
            // A Button 
      /*           new JoystickButton(armJoystick, 1).onTrue(
                     new SequentialCommandGroup( 
                          Commands.runOnce(()->swerveSubsystem.saveControllerThrottle()),
                          Commands.runOnce(()->swerveSubsystem.setThrottle(1)),
                          new DriveStraightCmd(swerveSubsystem, -.1, 0, 2),
                          Commands.runOnce(()->swerveSubsystem.restoreControllerThrottle())));

*/
           // DRIVE STRAIGHT COMMAND   
           /*      new JoystickButton(armJoystick, 1).onTrue(
                     new SequentialCommandGroup( 
                          Commands.runOnce(()->swerveSubsystem.saveControllerThrottle()),
                          Commands.runOnce(()->swerveSubsystem.setThrottle(1)),
                          new DriveStraightCmd(swerveSubsystem, -.1, 0, 2),
                          Commands.runOnce(()->swerveSubsystem.restoreControllerThrottle()))); */

           // ALIGN TO TAG COMMAND
            // A Button 
             new JoystickButton(armJoystick, 2).whileTrue(
                     new SequentialCommandGroup( 
                          Commands.runOnce(()->swerveSubsystem.saveControllerThrottle()),
                          Commands.runOnce(()->swerveSubsystem.setThrottle(1)),
                          //new TurnToPoseCmd(swerveSubsystem, TAG_POSITIONS[7].toPose2d(), 0, true),
                         
                          // new TurnToTagCmd(swerveSubsystem, VisionConstants.CAMERAS_TAG[0], 
                          //              VisionConstants.CAMERAS_TRANSFORM3DS_TAG[0], 7, 0, true),

                          new DriveToTagCmd(swerveSubsystem, VisionConstants.CAMERAS_TAG[0], 
                                        VisionConstants.CAMERAS_TRANSFORM3DS_TAG[0], 7, 2, 0, true)
                          
                          ));

        }    

   


        public Command getAutonomousCommand() {
              
                return  autoChooser.getSelected();
        }




        public Command getTeleOpInitCommand() {
                return new SequentialCommandGroup( 
                  //      Commands.runOnce(()->swerveSubsystem.setCurrentPose(new Pose2d(1.4, 5.6, new Rotation2d(Math.toRadians(FieldConstants.blueSpeakerMidAngle))))),
                //        Commands.runOnce(()->swerveSubsystem.setThrottle(0.4))
                 //       new ShooterWheelsCmd(shooterWheelsSubsystem, true),
                 //       new IntakeWheelsCmd(intakeWheelsSubsystem, 0, false)
                );
                        
        }

        public void teleopInit() {
                        ledSubsystem.setToTeamColor();
        }





  

}
