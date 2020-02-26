package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String AUTO_STOP = "Do nothing";
  private static final String AUTO_STRAIGHT = "Go Straight Forward";
  private static final String AUTO_RIGHT = "Turn Right";
  private static final String AUTO_LEFT = "Turn Left";
  private static final String AUTO_SHOOT_AND_RUN = "Shoot 5 balls then drive";

  //TODO: Tune this value
  private static final double GYRO_KP = 0.03;

  //6 inch wheels, 20 ticks per revolution, 10.71:1 gear ratio
  private static final double DISTANCE_PER_PULSE = Math.PI * .5 / (20.0 * 10.71); 

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Joystick leftStick  = new Joystick(0);
  Joystick rightStick = new Joystick(1);
  XboxController xbox = new XboxController(2);

  VictorSP leftDrive  = new VictorSP(0);
  VictorSP rightDrive = new VictorSP(1);
  VictorSP shooter = new VictorSP(2);
  VictorSP climber = new VictorSP(3);
  VictorSP intake = new VictorSP(4);
  VictorSP conveyor = new VictorSP(5);
  VictorSP elevator = new VictorSP(6);
  VictorSP agitator = new VictorSP(7);

  //DigitalOutput LEDRing = new DigitalOutput(0);
  
  DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  Encoder cimEncoder = new Encoder(0, 1, true);

  //Used to toggle intake on the right joystick
  boolean intakeToggle = false;
  boolean previous1 = false;
  boolean current1 = false;

  //Used to toggle intake on the left joystick
  boolean intakeToggle2 = false;
  boolean previous2 = false;
  boolean current2 = false;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Do Nothing", AUTO_STOP);
    m_chooser.addOption("Go Straight", AUTO_STRAIGHT);
    m_chooser.addOption("Go Right", AUTO_RIGHT);
    m_chooser.addOption("Go Left", AUTO_LEFT);
    m_chooser.addOption("Shoot and Move", AUTO_SHOOT_AND_RUN);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Set up sensors
    cimEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    gyro.calibrate();

    leftDrive.setInverted(true);
    rightDrive.setInverted(true);
    shooter.setInverted(true);
    conveyor.setInverted(true);
    intake.setInverted(true);

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);
  }

  @Override
  public void robotPeriodic() {
    int rawVal = cimEncoder.getRaw();
    SmartDashboard.putNumber("Gyro value: ", gyro.getAngle());
    SmartDashboard.putNumber("Encoder value", rawVal);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {

      case AUTO_STRAIGHT:
        if(cimEncoder.getDistance() < 5)
        {
          //If we are off from zero, steer proportionally the opposite of the gyro value
          double heading = -1 * gyro.getAngle() * GYRO_KP;
          drive.arcadeDrive(.5, heading);
        }
        else
        {
          drive.tankDrive(0.0, 0.0);
        }
        break;

      case AUTO_RIGHT:
        if(gyro.getAngle() < 90)
        {
          drive.tankDrive(.3,0);
          Timer.delay(0.01);
        }
        else
        {
          drive.tankDrive(0,0);
          Timer.delay(0.01);
        }
        break;

      case AUTO_LEFT:
        if(gyro.getAngle() > -90)
        {
          drive.tankDrive(0,.3);
          Timer.delay(0.01);
        }
        else
        {
          drive.tankDrive(0,0);
          Timer.delay(0.01);
        }
        break;

      case AUTO_SHOOT_AND_RUN:
        if(Timer.getMatchTime() > 14)
        {
          shooter.set(1);
          drive.tankDrive(0,0);
        }
        else if(Timer.getMatchTime() > 10)
        {
          shooter.set(1);
          elevator.set(1);
          conveyor.set(1);
          drive.tankDrive(0,0);
        }
        else if (cimEncoder.getDistance() < 5)
        {
          shooter.set(0);
          elevator.set(0);
          conveyor.set(0);
          drive.tankDrive(.7, .7);
          Timer.delay(0.01);
        }
        else
        {
          shooter.set(0);
          elevator.set(0);
          conveyor.set(0);
          drive.tankDrive(0,0);
        }
      
        break;

      case AUTO_STOP:
      default:
        drive.tankDrive(0,0);
        Timer.delay(0.01);
        break;
    }
  }

  @Override
  public void teleopPeriodic() {
    drive.tankDrive(leftStick.getRawAxis(1), rightStick.getRawAxis(1));

    boolean bothTriggersJoy = rightStick.getRawButton(1) && leftStick.getRawButton(1);
    boolean triggerXboxRight = xbox.getRawAxis(3) > 0;
    boolean triggerXboxLeft = xbox.getRawAxis(2) > 0;
    boolean a_button = xbox.getAButton();
    boolean b_button = xbox.getBButton();
    boolean x_button = xbox.getXButton();
    boolean y_button = xbox.getYButton();
    //boolean right_bumper = xbox.getBumper(Hand.kRight);
    //boolean left_bumper = xbox.getBumper(Hand.kLeft);

    /** Manipulate Shooter */
    if(bothTriggersJoy || triggerXboxRight || triggerXboxLeft) 
    {
      shooter.set(1);
    } 
    else 
    {
      shooter.set(0);
    }

    /** Manipulate Conveyor */
    if (bothTriggersJoy || b_button || triggerXboxLeft)
    {
      conveyor.set(.75);
    }
    else if(x_button)
    {
      conveyor.set(-1);
    } 
    else 
    {
      conveyor.set(0);
    }



    /** Manipulate Elevator */
    if (bothTriggersJoy || triggerXboxLeft || y_button) 
    {
      elevator.set(1);
    } 
    else if (a_button)
    {
      elevator.set(-1);
    }
    else 
    {
      elevator.set(0);
    }

    /** Set up toggle for intake */
    previous1 = current1;
    previous2 = current2;
    current1 = rightStick.getRawButton(2);
    current2 = leftStick.getRawButton(2);

    if (current1 && !previous1) 
    {
      intakeToggle = !intakeToggle;
    }

    if (current2 && !previous2) 
    {
      intakeToggle2 = !intakeToggle2;
    }

    if (intakeToggle) 
    {
      intake.set(1);
    }
    else if (intakeToggle2) 
    {
      intake.set(-1);
    } 
    else 
    {
      intake.set(0);
    }

    /** Manipulate agitator
     * Currently only runs when shooter or intake is running
     *
     */
    if(shooter.get() != 0 || intake.get() != 0)
    {
      agitator.set(1);
    }
    else
    {
      agitator.set(0);
    }
  }

  @Override
  public void testPeriodic() {
  }



}
