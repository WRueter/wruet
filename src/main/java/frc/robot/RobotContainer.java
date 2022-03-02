// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.OIConstants;
// // import frc.robot.commands.AcceptLimelightDistance;
// // import frc.robot.commands.DecrementShooterIndex;
// // import frc.robot.commands.IncrementShooterIndex;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
// // import frc.robot.commands.RunConveyorWithEncoder;
// // import frc.robot.commands.LiftInfeed;
// // import frc.robot.commands.ReverseInfeedAndConveyor;
// // import frc.robot.commands.RunConveyorOneBall;
// // import frc.robot.commands.RunConveyorTwoBall;
// // import frc.robot.commands.RunShooterMotors;
// // import frc.robot.commands.ToggleFineAdjustment;
// import frc.robot.commands.RunInfeedSingulatorMotors;
// import frc.robot.subsystems.Infeed;
// import frc.robot.subsystems.SingulatorAndInfeed;
// import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.SPEED_SCALE;

import java.time.Instant;
import java.util.List;
import java.util.function.BooleanSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DrivetrainSubsystem m_robotDrive = DrivetrainSubsystem.get_instance();
  // private final Infeed m_infeed = Infeed.get_instance();
  // private final SingulatorAndInfeed m_singulatorAndInfeed = SingulatorAndInfeed.get_instance();
  // private final RunInfeedSingulatorMotors _RunInfeedSingulatorMotors;
  private static RobotContainer _instance;
  private WaitCommand _wait;

  public static final RobotContainer get_instance(){
      if(_instance == null){
          _instance = new RobotContainer();
      }
      return _instance;
  }
//   private final Shooter m_shooter = Shooter.getInstance();




  // Controller Setup
  private BeakXBoxController m_driverController = new BeakXBoxController(0);
  private BeakXBoxController m_operatorController = new BeakXBoxController(1);
  //---

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    // _RunInfeedSingulatorMotors = new RunInfeedSingulatorMotors();
    // _wait = new WaitCommand(1.0);
    // _wait.addRequirements(m_singulatorAndInfeed);
    configureButtonBindings();

    RunCommand drivecommand = new RunCommand(
      () ->
          m_robotDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
              util.deadband(-m_driverController.getLeftYAxis()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              util.deadband(-m_driverController.getLeftXAxis()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              util.deadband(-m_driverController.getRightXAxis()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              m_robotDrive.getGyroscopeRotation()
              )));

    drivecommand.addRequirements(m_robotDrive);

    // Configure default commands
    m_robotDrive.setDefaultCommand(drivecommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      m_driverController.start.whenPressed(new InstantCommand(() -> m_robotDrive.zeroGyroscope()));
      /*m_driverController.left_bumper.whenPressed(new InstantCommand(() -> 
      m_singulatorAndInfeed.liftInfeed())
      .andThen(new WaitCommand(2.0))
      .andThen(new InstantCommand(() -> m_singulatorAndInfeed.holdInfeed())));
      m_driverController.right_bumper.whenPressed(new InstantCommand(() -> 
      m_singulatorAndInfeed.downInfeed())
      .andThen(new WaitCommand(1.0))
      .andThen(new InstantCommand(() -> m_singulatorAndInfeed.holdInfeed())));*/
      // m_driverController.y.toggleWhenPressed(_RunInfeedSingulatorMotors);
    //   m_operatorController.b.whenPressed(new RunConveyorWithEncoder());
    // //   m_operatorController.x.toggleWhenPressed(new RunShooterMotors());
    //   m_operatorController.a.whenPressed(new RunConveyorTwoBall());
    //   m_operatorController.start.toggleWhenPressed(new RunConveyorOneBall());
    // //   m_operatorController.right_bumper.whenPressed(new InstantCommand(() -> m_shooter.shiftShooterVbus(0, 0.02)));
    // //   m_operatorController.left_bumper.whenPressed(new InstantCommand(() -> m_shooter.shiftShooterVbus(0.02, 0)));
    //   m_operatorController.back.toggleWhenPressed(new ReverseInfeedAndConveyor());
    //   m_operatorController.left_bumper.whenPressed(new DecrementShooterIndex());
    //   m_operatorController.right_bumper.whenPressed(new IncrementShooterIndex());
    //   m_operatorController.left_stick_button.whenPressed(new ToggleFineAdjustment());
    //   m_operatorController.right_stick_button.whenPressed(new AcceptLimelightDistance());
      // FIXME: bruh spagheti controller

  }

  public double getRightTrigger(){
    return m_driverController.getRightTrigger();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainSubsystem.m_kinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(3, 0, new Rotation2d(Math.PI / 2))),
            config);

    var thetaController =
        new ProfiledPIDController(
            3.0, 0, 0, new Constraints(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DrivetrainSubsystem.m_kinematics,

            // Position controllers
            new PIDController(2.00, 0, 0),
            new PIDController(2.00, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand
    .andThen(() -> m_robotDrive.drive(new ChassisSpeeds()))
    .andThen(new InstantCommand(() -> System.out.println("~~~~AUTON COMPLETE~~~~")))
    .andThen(new InstantCommand(() -> System.out.println(m_robotDrive.getPose().toString())));
  }

}
