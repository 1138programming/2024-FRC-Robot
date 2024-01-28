// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.subsystems.Base;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer {
  //Controller Ports (check in Driver Station, IDs may be different for each computer)
  private static final int KLogitechPort = 0;
  private static final int KXboxPort = 1;  
  private static final int KStreamDeckPort = 2;
  private static final int KTestingStreamDeckPort = 3;
  private static final int KTuningStreamDeckPort = 4;

  //Deadzone
  private static final double KDeadZone = 0.05;
  
  //Joystick Axis IDs 
  private static final int KLeftXAxis = 0;
  private static final int KLeftYAxis = 1;
  private static final int KRightXAxis = 2;
  private static final int KRightYAxis = 3;

  //Joystick Axis IDs 
  private static final int KXboxLeftYAxis = 1;
  private static final int KXboxRightYAxis = 5;
  private static final int KXboxLeftXAxis = 0;
  private static final int KXboxRightXAxis = 4;

  //Logitech Button Constants
  public static final int KLogitechButtonX = 1;
  public static final int KLogitechButtonA = 2;
  public static final int KLogitechButtonB = 3;
  public static final int KLogitechButtonY = 4;
  public static final int KLogitechLeftBumper = 5;
  public static final int KLogitechRightBumper = 6;
  public static final int KLogitechLeftTrigger = 7;
  public static final int KLogitechRightTrigger = 8;

  //Xbox Button Constants
  public static final int KXboxButtonA = 1;
  public static final int KXboxButtonB = 2;
  public static final int KXboxButtonX = 3;
  public static final int KXboxButtonY = 4;
  public static final int KXboxLeftBumper = 5;
  public static final int KXboxRightBumper = 6;
  public static final int KXboxSelectButton = 7;
  public static final int KXboxStartButton = 8;
  public static final int KXboxLeftTrigger = 2;
  public static final int KXboxRightTrigger = 3;

  //Game Controllers
  public static Joystick logitech;
  public static Joystick compStreamDeck;
  public static Joystick testStreamDeck;
  public static Joystick tuningStreamDeck;
  public static XboxController xbox; 
  //Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button

  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

  public Trigger xboxBtnRT, xboxBtnLT;
  
  public JoystickButton comp1, comp2, comp3, comp4, comp5, comp6, comp7, comp8, comp9, comp10, comp11, comp12, comp13, comp14;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton streamDeck1, streamDeck2, streamDeck3, streamDeck4, streamDeck5, streamDeck6, streamDeck7, streamDeck8, streamDeck9, // Vjoy 2
    streamDeck10, streamDeck11, streamDeck12, streamDeck13, streamDeck14, streamDeck15;
   
  // Subsystems
  private final Base base = new Base();
  // Commands
  private final DriveWithJoysticks drivewithJoysticks = new DriveWithJoysticks(base);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    base.setDefaultCommand(drivewithJoysticks);
    // Configure the trigger bindings


    logitech = new Joystick(KLogitechPort); //Logitech Dual Action
    xbox = new XboxController(KXboxPort);   //Xbox 360 for Windows
    compStreamDeck = new Joystick(KStreamDeckPort);   //Stream Deck + vjoy
    testStreamDeck = new Joystick(KTestingStreamDeckPort);   //Stream Deck + vjoy
    tuningStreamDeck = new Joystick(KTuningStreamDeckPort);   //Stream Deck + vjoy

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

    comp1 = new JoystickButton(compStreamDeck, 1);
    comp2 = new JoystickButton(compStreamDeck, 2);
    comp3 = new JoystickButton(compStreamDeck, 3);
    comp4 = new JoystickButton(compStreamDeck, 4);
    comp5 = new JoystickButton(compStreamDeck, 5);
    comp6 = new JoystickButton(compStreamDeck, 6);
    comp7 = new JoystickButton(compStreamDeck, 7);
    comp8 = new JoystickButton(compStreamDeck, 8);
    comp9 = new JoystickButton(compStreamDeck, 9);
    comp10 = new JoystickButton(compStreamDeck, 10);
    comp11 = new JoystickButton(compStreamDeck, 11);
    comp12 = new JoystickButton(compStreamDeck, 12);
    comp13 = new JoystickButton(compStreamDeck, 13);
    comp14 = new JoystickButton(compStreamDeck, 14);

    streamDeck1 = new JoystickButton(testStreamDeck, 1);
    streamDeck2 = new JoystickButton(testStreamDeck, 2);
    streamDeck3 = new JoystickButton(testStreamDeck, 3);
    streamDeck4 = new JoystickButton(testStreamDeck, 4);
    streamDeck5 = new JoystickButton(testStreamDeck, 5);
    streamDeck6 = new JoystickButton(testStreamDeck, 6);
    streamDeck7 = new JoystickButton(testStreamDeck, 7);
    streamDeck8 = new JoystickButton(testStreamDeck, 8);
    streamDeck9 = new JoystickButton(testStreamDeck, 9);
    streamDeck10 = new JoystickButton(testStreamDeck, 10);
    streamDeck11 = new JoystickButton(testStreamDeck, 11);
    streamDeck12 = new JoystickButton(testStreamDeck, 12);
    streamDeck13 = new JoystickButton(testStreamDeck, 13);
    streamDeck14 = new JoystickButton(testStreamDeck, 14);
    streamDeck15 = new JoystickButton(testStreamDeck, 15);
  	
    // Configure the button bindings
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
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
    if(Y > KDeadZone || Y < -KDeadZone)
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
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KXboxLeftXAxis);
    if(X > KDeadZone || X < -KDeadZone)
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
    if(Y > KDeadZone || Y < -KDeadZone)
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

