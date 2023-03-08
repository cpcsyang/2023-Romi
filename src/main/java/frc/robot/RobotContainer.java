// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.RunMotorSim;
import frc.robot.commands.TurnTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MotorSim;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final OnBoardIO onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  private RunMotorSim runMotorSim;
  private MotorSim motorSim = new MotorSim();

  // Assumes a gamepad plugged into channnel 0
  // private final Joystick m_controller = new Joystick(0);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("onBoardIO Button A Pressed"))
        .onFalse(new PrintCommand("onBoardIO Button A Released"));

    // Example of how to use the controller buttons
    // driverController.a()
    //     .onTrue(new InstantCommand(() -> System.out.println("A button pressed")))
    //     .onFalse(new InstantCommand(() -> System.out.println("A button released")));
    driverController.a()
            .onTrue(new ParallelCommandGroup(new PrintCommand("1.. "))
            .alongWith(new PrintCommand("2.. ")).alongWith(new PrintCommand("3.. ")));
    driverController.b()
            .onTrue(new SequentialCommandGroup(new TurnTime(0.5, 5, drivetrain))
            .andThen(new TurnTime(-0.5, 5, drivetrain))
            .andThen(new TurnTime(0.2, 10, drivetrain)));
        driverController.leftBumper().and(driverController.rightBumper())
        .onTrue(new InstantCommand(() -> System.out.println("Both L+R bumpers pressed")))
        .onFalse(new InstantCommand(() -> System.out.println("Both L+R bumpers released")));

    driverController.x().or(driverController.y())
        .onTrue(new InstantCommand(() -> System.out.println("X or Y pressed")))
        .onFalse(new InstantCommand(() -> System.out.println("X or Y released")));
    double squeezeThreshold = 0.25;
    driverController.rightTrigger(squeezeThreshold)
        .onTrue(new InstantCommand(() -> System.out.println("Right Trigger squeezed > " + squeezeThreshold)))
        .onFalse(new InstantCommand(() -> System.out.println("Right Trigger released")));
    driverController.leftStick()
        .onTrue(new InstantCommand(() -> System.out.println("Left Stick Clicked")))
        .onFalse(new InstantCommand(() -> System.out.println("Left Stick Released")));
    driverController.b().and(driverController.povUp())
        .onTrue(new InstantCommand(() -> System.out.println("B + povUP pressed")))
        .onFalse(new InstantCommand(() -> System.out.println("B + povUP released")));
    driverController.b().and(driverController.povLeft())
        .onTrue(new InstantCommand(() -> System.out.println("B + povLEFT pressed")))
        .onFalse(new InstantCommand(() -> System.out.println("B + povLEFT released")));
    driverController.b().and(driverController.povDown())
        .onTrue(new InstantCommand(() -> System.out.println("B + povDOWN pressed")))
        .onFalse(new InstantCommand(() -> System.out.println("B + povDOWN released")));
    driverController.b().and(driverController.povRight())
        .onTrue(new InstantCommand(() -> System.out.println("B + povRIGHT pressed")))
        .onFalse(new InstantCommand(() -> System.out.println("B + povRIGHT released")));

    // Setup SmartDashboard options
    chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(drivetrain));
    chooser.addOption("Auto Routine Time", new AutonomousTime(drivetrain));
    SmartDashboard.putData(chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        // m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
        drivetrain, () -> -driverController.getLeftY(), () -> -driverController.getRightX());
  }
}
