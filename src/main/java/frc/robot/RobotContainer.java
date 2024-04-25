// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.Trap;
import frc.robot.commands.Base.AmpDrivingMode;
// Commands
//  Base
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.RotateToSpeaker;
import frc.robot.commands.Base.SpeakerDrivingMode;
import frc.robot.commands.Base.ToggleLimelight;
import frc.robot.commands.Base.ToggleSpeed;
import frc.robot.commands.Flywheel.FlywheelSetBrakeMode;
import frc.robot.commands.Flywheel.FlywheelSetCoastMode;
//  Flywheel
import frc.robot.commands.Flywheel.SpinFlywheel;
import frc.robot.commands.Flywheel.SpinFlywheelAndRotate;
import frc.robot.commands.Flywheel.SpinFlywheelAndTilt;
import frc.robot.commands.Flywheel.SpinFlywheelAndTiltLow;
import frc.robot.commands.Flywheel.SpinFlywheelBottom;
import frc.robot.commands.Flywheel.SpinFlywheelFullSpeed;
import frc.robot.commands.Flywheel.SpinFlywheelReverse;
import frc.robot.commands.Flywheel.SpinFlywheelSlow;
import frc.robot.commands.Flywheel.SpinLowerFlywheel;
import frc.robot.commands.Flywheel.SpinUpperFlywheel;
import frc.robot.commands.Flywheel.StopFlywheel;
import frc.robot.commands.Flywheel.ThroughBore.SpinFlywheelAmp;
import frc.robot.commands.Flywheel.ThroughBore.SpinFlywheelAutoAim;
import frc.robot.commands.Flywheel.ThroughBore.SpinFlywheelSpeaker;
import frc.robot.commands.Flywheel.ThroughBore.SpinFlywheelSpeakerAuton2;
import frc.robot.commands.Flywheel.ThroughBore.SpinFlywheelSpeakerPodium;
import frc.robot.commands.Hang.MoveHangDown;
import frc.robot.commands.Hang.MoveHangUp;
import frc.robot.commands.Hang.ToggleHangPistons;
import frc.robot.commands.Hang.StopHang;
//  Indexer
import frc.robot.commands.Indexer.IndexerLoadNote;
import frc.robot.commands.Indexer.IndexerSpin;
import frc.robot.commands.Indexer.IndexerSpinBack;
import frc.robot.commands.Indexer.IndexerStop;
//  Intake
import frc.robot.commands.Intake.IntakeSpinIn;
import frc.robot.commands.Intake.IntakeSpinOut;
import frc.robot.commands.Intake.IntakeSpinStop;
import frc.robot.commands.ShooterTilt.AutoAimShooterTilt;
//  ShooterTilt
import frc.robot.commands.ShooterTilt.MoveShooterTiltToPos;
import frc.robot.commands.ShooterTilt.MoveShooterTiltTop;
import frc.robot.commands.ShooterTilt.SetManualControl;
import frc.robot.commands.ShooterTilt.ShooterTiltSpinDown;
import frc.robot.commands.ShooterTilt.ShooterTiltSpinUp;
import frc.robot.commands.ShooterTilt.ShooterTiltStop;
import frc.robot.commands.ShooterTilt.ShooterTiltWait;
import frc.robot.CommandGroups.AutonShoot;
import frc.robot.CommandGroups.AutonShootLow;
import frc.robot.CommandGroups.IndexAndShoot;
// Command Groups+++
import frc.robot.CommandGroups.IntakeAndIndex;
import frc.robot.CommandGroups.IntakeAndIndexOut;
import frc.robot.CommandGroups.IntakeAndIndexToStop;
import frc.robot.CommandGroups.IntakeAutonStopFlywheel;

// Constants
import static frc.robot.Constants.SwerveDriveConstants.*;
import static frc.robot.Constants.OperatorConstants.*;

//WPILib
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//Trap Imports

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // Subsystems
  private final Base base = new Base();
  private final Intake intake = new Intake();
  private final ShooterTilt shooterTilt = new ShooterTilt();
  private final Flywheel flyWheel = new Flywheel();
  private final Indexer indexer = new Indexer();
  private final Hang hang = new Hang();
  private final LEDs leds = new LEDs();

  // private final Trap trap = new Trap();
  // Commands
  //  Swerve Drive Commands
  private final DriveWithJoysticks drivewithJoysticks = new DriveWithJoysticks(base);
  private final SetManualControl setManualControl = new SetManualControl(shooterTilt);
  private final ToggleSpeed toggleMaxSpeed = new ToggleSpeed(base, KBaseDriveMaxPercent, KBaseRotMaxPercent);
  private final ToggleSpeed toggleMidSpeed = new ToggleSpeed(base, KBaseDriveMidPercent, KBaseRotMidPercent);
  private final ToggleSpeed toggleLowSpeed = new ToggleSpeed(base, KBaseDriveLowPercent, KBaseRotLowPercent);
  private final ResetGyro resetGyro = new ResetGyro(base);
  private final RotateToSpeaker rotateToSpeaker = new RotateToSpeaker(base);
  private final AmpDrivingMode ampDrivingMode = new AmpDrivingMode(base, hang);
  private final SpeakerDrivingMode speakerDrivingMode = new SpeakerDrivingMode(base);
  private final ToggleLimelight toggleLimelight = new ToggleLimelight();
  //  Shooter Tilt Commands
  private final ShooterTiltStop shooterTiltStop = new ShooterTiltStop(shooterTilt);
  private final MoveShooterTiltToPos moveShooterTiltToPos = new MoveShooterTiltToPos(shooterTilt, 0);
  private final ShooterTiltWait shooterTiltWait = new ShooterTiltWait(shooterTilt, leds);
  private final MoveShooterTiltToPos moveShooterTiltTo90 = new MoveShooterTiltToPos(shooterTilt, 90);
  private final ShooterTiltSpinDown shooterTiltSpinDown = new ShooterTiltSpinDown(shooterTilt);
  private final ShooterTiltSpinUp shooterTiltSpinUp = new ShooterTiltSpinUp(shooterTilt);
  private final MoveShooterTiltTop moveShooterTiltTop = new MoveShooterTiltTop(shooterTilt);
  private final AutoAimShooterTilt autoAimShooterTilt = new AutoAimShooterTilt(shooterTilt);
  //  Intake Commands
  private final IntakeSpinStop intakeSpinStop = new IntakeSpinStop(intake);
  private final IntakeSpinIn intakeSpinIn = new IntakeSpinIn(intake);
  private final IntakeSpinOut intakeSpinOut = new IntakeSpinOut(intake);
  //  Flywheel Commands
  private final StopFlywheel stopFlywheel = new StopFlywheel(flyWheel);
  private final SpinFlywheel spinFlywheel = new SpinFlywheel(flyWheel);
  private final SpinUpperFlywheel spinUpperFlywheel = new SpinUpperFlywheel(flyWheel);
  private final SpinLowerFlywheel spinLowerFlywheel = new SpinLowerFlywheel(flyWheel);
  private final SpinFlywheelSpeaker spinFlywheelSpeaker = new SpinFlywheelSpeaker(flyWheel, shooterTilt);
  private final SpinFlywheelSpeakerAuton2 spinFlywheelSpeakerAuton2 = new SpinFlywheelSpeakerAuton2(flyWheel, shooterTilt);
  private final SpinFlywheelSpeakerPodium spinFlywheelSpeakerPodium = new SpinFlywheelSpeakerPodium(flyWheel, shooterTilt);
  private final SpinFlywheelFullSpeed spinFlywheelFullSpeed = new SpinFlywheelFullSpeed(flyWheel);
  private final SpinFlywheelAmp spinFlywheelAmp = new SpinFlywheelAmp(flyWheel, shooterTilt);
  private final SpinFlywheelAutoAim spinFlywheelAutoAim = new SpinFlywheelAutoAim(flyWheel, shooterTilt);
  private final SpinFlywheelReverse spinFlywheelReverse = new SpinFlywheelReverse(flyWheel);
  private final SpinFlywheelBottom spinFlywheelBottom = new SpinFlywheelBottom(flyWheel, shooterTilt);

  private final SpinFlywheelAndRotate spinFlywheelAndRotate = new SpinFlywheelAndRotate(flyWheel, base, shooterTilt);
  private final SpinFlywheelAndTilt spinFlywheelAndTilt = new SpinFlywheelAndTilt(flyWheel, shooterTilt);
  private final SpinFlywheelAndTiltLow spinFlywheelAndTiltLow = new SpinFlywheelAndTiltLow(flyWheel, shooterTilt);
  private final SpinFlywheelSlow spinFlywheelSlow = new SpinFlywheelSlow(flyWheel);

  private final FlywheelSetBrakeMode flywheelSetBrakeMode = new FlywheelSetBrakeMode(flyWheel);
  private final FlywheelSetCoastMode flywheelSetCoastMode = new FlywheelSetCoastMode(flyWheel);
  //  Indexer Commands
  private final IndexerSpin indexerSpin = new IndexerSpin(indexer);
  private final IndexerSpinBack indexerSpinBack = new IndexerSpinBack(indexer);
  private final IndexerStop indexerStop = new IndexerStop(indexer);
  //  Hang Commands
  private final ToggleHangPistons toggleHangPistons = new ToggleHangPistons(hang);
  private final StopHang stopHang = new StopHang(hang);
  
  // Command Groups
  // private final ShootAnd ampAutoShoot = new ShootAnd(base, flyWheel, shooterTilt, indexer);
  private final IntakeAndIndex intakeAndIndex = new IntakeAndIndex(intake, indexer);
  private final IntakeAndIndexOut intakeAndIndexOut = new IntakeAndIndexOut(intake, indexer);
  private final IntakeAndIndexToStop intakeAndIndexToStop = new IntakeAndIndexToStop(intake, indexer);
  private final IndexAndShoot indexAndShoot = new IndexAndShoot(flyWheel, indexer);
  private final AutonShoot autonShoot = new AutonShoot(indexer, flyWheel, shooterTilt);
  private final AutonShootLow autonShootLow = new AutonShootLow(indexer, flyWheel, shooterTilt);
  private final IntakeAutonStopFlywheel intakeAutonStopFlywheel = new IntakeAutonStopFlywheel(intake, indexer, shooterTilt, flyWheel);

  // Shuffleboard AutonChooser
  private final SendableChooser<Command> autonChooser;

  // Game Controllers
  public static Joystick logitech;
  public static Joystick compStreamDeck;
  public static Joystick testStreamDeck;
  public static Joystick autonTestStreamDeck;
  public static XboxController xbox;
  
  // Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB,
      logitechBtnLT, logitechBtnRT; // Logitech Button

  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

  public Trigger xboxBtnRT, xboxBtnLT;

  public JoystickButton compStreamDeck1, compStreamDeck2, compStreamDeck3, compStreamDeck4, compStreamDeck5,
      compStreamDeck6, compStreamDeck7, compStreamDeck8, compStreamDeck9, compStreamDeck10, compStreamDeck11,
      compStreamDeck12, compStreamDeck13,
      compStreamDeck14, compStreamDeck15;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton testStreamDeck1, testStreamDeck2, testStreamDeck3, testStreamDeck4, testStreamDeck5,
      testStreamDeck6, testStreamDeck7,
      testStreamDeck8, testStreamDeck9, // Vjoy 2
      testStreamDeck10, testStreamDeck11, testStreamDeck12, testStreamDeck13, testStreamDeck14, testStreamDeck15;

  public JoystickButton autonTestStreamDeck1, autonTestStreamDeck2, autonTestStreamDeck3, autonTestStreamDeck4,
      autonTestStreamDeck5, autonTestStreamDeck6, autonTestStreamDeck7,
      autonTestStreamDeck8, autonTestStreamDeck9, // Vjoy 2
      autonTestStreamDeck10, autonTestStreamDeck11, autonTestStreamDeck12, autonTestStreamDeck13, autonTestStreamDeck14,
      autonTestStreamDeck15;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    base.setDefaultCommand(drivewithJoysticks);
    intake.setDefaultCommand(intakeSpinStop);
    shooterTilt.setDefaultCommand(shooterTiltWait);
    indexer.setDefaultCommand(indexerStop);
    flyWheel.setDefaultCommand(stopFlywheel);
    hang.setDefaultCommand(stopHang);
    
    // Named commands for PathPlanner
    NamedCommands.registerCommand("rotateToSpeaker", rotateToSpeaker);
    NamedCommands.registerCommand("shooterTiltWait", shooterTiltWait);
    NamedCommands.registerCommand("stopFlywheel", stopFlywheel);
    NamedCommands.registerCommand("intakeAndIndexToStop", intakeAndIndexToStop);
    NamedCommands.registerCommand("indexAndShoot", indexAndShoot);
    NamedCommands.registerCommand("spinFlywheelSpeaker", spinFlywheelSpeaker);
    NamedCommands.registerCommand("spinFlywheelSpeakerPodium", spinFlywheelSpeakerPodium);
    NamedCommands.registerCommand("spinFlywheelSpeakerAuton2", spinFlywheelSpeakerAuton2);
    NamedCommands.registerCommand("indexerSpin", indexerSpin);
    NamedCommands.registerCommand("stopFlywheel", stopFlywheel);
    NamedCommands.registerCommand("rotateToSpeaker", new RotateToSpeaker(base));
    NamedCommands.registerCommand("intakeAndIndex", intakeAndIndex);
    NamedCommands.registerCommand("indexerSpinBack", indexerSpinBack);
    NamedCommands.registerCommand("autoAimShooterTilt", autoAimShooterTilt);
    NamedCommands.registerCommand("spinFlywheelAndRotate", spinFlywheelAndRotate);
    NamedCommands.registerCommand("spinFlywheelAndTilt", spinFlywheelAndTilt);
    NamedCommands.registerCommand("spinFlywheelAndTiltLow", spinFlywheelAndTiltLow);
    NamedCommands.registerCommand("autonShoot", autonShoot);
    NamedCommands.registerCommand("spinFlywheelSlow", spinFlywheelSlow);
    NamedCommands.registerCommand("intakeAutonStopFlywheel", intakeAutonStopFlywheel);
    NamedCommands.registerCommand("autonShootLow", autonShootLow);
    NamedCommands.registerCommand("flywheelSetBrakeMode", flywheelSetBrakeMode);

    // Auton Chooser
    autonChooser = AutoBuilder.buildAutoChooser("4 NOTE Mid");
    SmartDashboard.putData("Auton Chooser", autonChooser);


    // Configure the trigger bindings
    logitech = new Joystick(KLogitechPort); // Logitech Dual Action
    xbox = new XboxController(KXboxPort); // Xbox 360 for Windows
    compStreamDeck = new Joystick(KCompStreamDeckPort); // Stream Deck + vjoy
    testStreamDeck = new Joystick(KTestingStreamDeckPort); // Stream Deck + vjoy
    autonTestStreamDeck = new Joystick(KAutonTestingStreamDeckPort); // Stream Deck + vjoy

    // Logitch Buttons
    logitechBtnX = new JoystickButton(logitech, KLogitechButtonX);
    logitechBtnA = new JoystickButton(logitech, KLogitechButtonA);
    logitechBtnB = new JoystickButton(logitech, KLogitechButtonB);
    logitechBtnY = new JoystickButton(logitech, KLogitechButtonY);
    logitechBtnLB = new JoystickButton(logitech, KLogitechLeftBumper);
    logitechBtnRB = new JoystickButton(logitech, KLogitechRightBumper);
    logitechBtnLT = new JoystickButton(logitech, KLogitechLeftTrigger);
    logitechBtnRT = new JoystickButton(logitech, KLogitechRightTrigger);

    // XBox Buttons
    xboxBtnA = new JoystickButton(xbox, KXboxButtonA);
    xboxBtnB = new JoystickButton(xbox, KXboxButtonB);
    xboxBtnX = new JoystickButton(xbox, KXboxButtonX);
    xboxBtnY = new JoystickButton(xbox, KXboxButtonY);
    xboxBtnLB = new JoystickButton(xbox, KXboxLeftBumper);
    xboxBtnRB = new JoystickButton(xbox, KXboxRightBumper);
    xboxBtnSelect = new JoystickButton(xbox, KXboxSelectButton);
    xboxBtnStrt = new JoystickButton(xbox, KXboxStartButton);
    xboxBtnLT = new Trigger(() -> (joystickThreshold(xbox.getRawAxis(KXboxLeftTrigger))));
    xboxBtnRT = new Trigger(() -> (joystickThreshold(xbox.getRawAxis(KXboxRightTrigger))));

    compStreamDeck1 = new JoystickButton(compStreamDeck, 1);
    compStreamDeck2 = new JoystickButton(compStreamDeck, 2);
    compStreamDeck3 = new JoystickButton(compStreamDeck, 3);
    compStreamDeck4 = new JoystickButton(compStreamDeck, 4);
    compStreamDeck5 = new JoystickButton(compStreamDeck, 5);
    compStreamDeck6 = new JoystickButton(compStreamDeck, 6);
    compStreamDeck7 = new JoystickButton(compStreamDeck, 7);
    compStreamDeck8 = new JoystickButton(compStreamDeck, 8);
    compStreamDeck9 = new JoystickButton(compStreamDeck, 9);
    compStreamDeck10 = new JoystickButton(compStreamDeck, 10);
    compStreamDeck11 = new JoystickButton(compStreamDeck, 11);
    compStreamDeck12 = new JoystickButton(compStreamDeck, 12);
    compStreamDeck13 = new JoystickButton(compStreamDeck, 13);
    compStreamDeck14 = new JoystickButton(compStreamDeck, 14);
    compStreamDeck15 = new JoystickButton(compStreamDeck, 15);

    testStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    testStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    testStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    testStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    testStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    testStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    testStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    testStreamDeck8 = new JoystickButton(testStreamDeck, 8);
    testStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    testStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    testStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    testStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    testStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    testStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    testStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    autonTestStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    autonTestStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    autonTestStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    autonTestStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    autonTestStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    autonTestStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    autonTestStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    autonTestStreamDeck8 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    autonTestStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    autonTestStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    autonTestStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    autonTestStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    autonTestStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*      Driver Controls     */
    //  Auto aim while driving
    // logitechBtnRB.whileTrue(speakerDrivingMode);
    // logitechBtnRT.whileTrue(ampDrivingMode);
    //  Intake
    logitechBtnRB.whileTrue(intakeAndIndexToStop);
    logitechBtnRT.whileTrue(intakeAndIndexOut);
    // Other
    logitechBtnY.onTrue(resetGyro);
    logitechBtnA.whileTrue(spinFlywheelFullSpeed);
    logitechBtnX.whileTrue(speakerDrivingMode);
    logitechBtnLB.onTrue(toggleMaxSpeed);
    logitechBtnLT.onTrue(speakerDrivingMode);
    // Controlling Speed
    //  if LB and RB are held and one is released, go back to previous speed
    if (!logitechBtnLB.getAsBoolean()) {
      logitechBtnLT.onFalse(toggleMidSpeed);
    } else {
      logitechBtnLT.onFalse(toggleMaxSpeed);
    } 
    if (!logitechBtnLT.getAsBoolean()) {
      logitechBtnLB.onFalse(toggleMidSpeed);
    } else {
      logitechBtnLB.onFalse(toggleLowSpeed);
    }

    /*    Comp Arms and Lifts Controls      */
    //1-5
    compStreamDeck1.onTrue(spinFlywheelSpeaker);
    compStreamDeck2.onTrue(spinFlywheelSpeakerPodium);
    compStreamDeck3.whileTrue(shooterTiltSpinUp);
    compStreamDeck4.onTrue(stopFlywheel);
    compStreamDeck5.whileTrue(indexerSpin);
    //6-10
    compStreamDeck6.onTrue(spinFlywheelAmp);
    compStreamDeck7.whileTrue(spinFlywheelAndRotate);
    compStreamDeck8.whileTrue(shooterTiltSpinDown);
    compStreamDeck9.onTrue(toggleHangPistons);
    compStreamDeck10.whileTrue(indexerSpinBack);
    //11-15
    compStreamDeck11.whileTrue(intakeAndIndexToStop);
    compStreamDeck12.whileTrue(spinFlywheelReverse);
    compStreamDeck13.onTrue(setManualControl);
    compStreamDeck14.whileTrue(spinFlywheel);
    compStreamDeck15.whileTrue(intakeAndIndexOut);
    
    // Testing Arms and Lifts Controls
    testStreamDeck1.whileTrue(rotateToSpeaker);
    testStreamDeck2.whileTrue(spinFlywheelAutoAim);
    testStreamDeck3.onTrue(setManualControl);
    
    testStreamDeck8.whileTrue(intakeSpinOut);
    
    testStreamDeck11.whileTrue(spinFlywheel);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new SubBackPickShootAuton(base, flyWheel, shooterTilt, indexer, intake);
    return autonChooser.getSelected();
  }

  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    SmartDashboard.putNumber("getLogiRightYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    SmartDashboard.putNumber("getLogiLeftYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    SmartDashboard.putNumber("getLogiRightXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    SmartDashboard.putNumber("getLogiLeftXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getXboxLeftAxis() {
    final double Y = xbox.getRawAxis(KXboxLeftYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KXboxLeftXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return X;
    else
      return 0;
  }

  public double getXboxRightXAxis() {
    final double X = xbox.getRawAxis(KXboxRightXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return -X;
    else
      return 0;
  }

  public double getXboxLeftYAxis() {
    final double Y = xbox.getRawAxis(KXboxLeftYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getXboxRightYAxis() {
    final double Y = xbox.getRawAxis(KXboxRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public boolean joystickThreshold(double triggerValue) {
    if (Math.abs(triggerValue) < .09)
      return false;
    else
      return true;
  }
}