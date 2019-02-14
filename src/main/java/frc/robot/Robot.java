/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.DriveBase.DriveType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static int pigeonPort = RobotMap.PIGEONIMU;
  private static Pigeon pigeon = new Pigeon(pigeonPort);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private Arduino arduino = new Arduino();
  private ProximitySensor leftDistance = new ProximitySensor(arduino, RobotMap.LEFT_DISTANCE_SENSOR);
  private ProximitySensor rightDistance = new ProximitySensor(arduino, RobotMap.RIGHT_DISTANCE_SENSOR);
  private DriverIF controller = new DriverIF();
  private Navx navx = new Navx(Navx.Port.I2C);
  private DriveBase base = new DriveBase(controller, navx, pigeon, DriveType.Tank);
  private Hatch hatch = new Hatch(controller, base);
  private proximityEnum proximityState = proximityEnum.LookingForID;
  private Relay visionRelay = new Relay(0);

  private enum proximityEnum {
    LookingForID, LookingForData, Error
  }
  private AutoMaster auto = new AutoMaster(base, navx, hatch);

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("right kP", 0.01);
    SmartDashboard.putNumber("left kP", 0.01);
    SmartDashboard.putNumber("right kI", 0.001);
    SmartDashboard.putNumber("left kI", 0.001);
    SmartDashboard.putNumber("right Izone", 30);
    SmartDashboard.putNumber("left Izone", 30);
    SmartDashboard.putNumber("right kD", 10);
    SmartDashboard.putNumber("left kD", 10);
    visionRelay.set(Relay.Value.kForward);
    auto.start();
  }

  @Override
  public void robotPeriodic() {
    /*
    Scheduler.getInstance().run();

    byte[] dataMessage = arduino.readSerialPort();

    if (dataMessage.length == 2) {
      int dataByte0 = (int) (Byte.toUnsignedInt(dataMessage[0]));
      int dataByte1 = (int) (Byte.toUnsignedInt(dataMessage[1]));

      switch (proximityState) {

      case LookingForID:

        System.out.println("DataByte0: " + dataByte0);
        System.out.println("DataByte1 " + dataByte1);

        if (dataByte0 == 0xff) {
          if (dataByte1 == 0x01 || dataByte1 == 0x2) {
            proximityState = proximityEnum.LookingForData;
          }
        }

        break;

      case LookingForData:

        System.out.println("Proximity Sensor " + leftDistance.distanceInches() + "--------------------------");
        proximityState = proximityEnum.LookingForID;

        break;

      case Error:
      default:

        break;

      }

    }*/

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    base.teleopInit();
    auto.start();
  }

  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopInit() {
    base.teleopInit();
    pigeon.resetYaw();

  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    base.TeleopMove();
    // pixy.read();
    hatch.teleop();
    // System.out.println(distance.distanceInches());

    // pigeon.pigeonCheck();
    // System.out.println(navx.getYaw());

    // PixyData p = new PixyData();
    /*
     * try { p = pixy.readPacket(1); if(p == null) p = new PixyData(); }
     * catch(Exception e) { e.printStackTrace(); } System.out.println("X: " + p.X +
     * "Y: " + p.Y + "Width: " + p.Width + "Height: " + p.Height);
     */

    // pixy.read();
  }
  @Override
  public void testInit() {
    base.autoInit();
  }
  @Override
  public void testPeriodic() {
    //base.teleopInit();
    //base.rightFront.set(ControlMode.Position, 10000);
    //System.out.println("Counts: " + base.rightFront.getSensorPosition());
    //System.out.println("hello fam: " + base.rightFront.getSensorVelocity());
    base.TestEncoders();
    //hatch.testPeriodic();
  }
  
}