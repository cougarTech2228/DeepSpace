/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.ILoopable;
import frc.robot.LEDUtilities.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.DriveBase.DriveType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import com.ctre.phoenix.schedulers.ConcurrentScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";

  private SerialDataHandler serialDataHandler = new SerialDataHandler();

  // private static int pigeonPort = RobotMap.PIGEONIMU;
  // private static Pigeon pigeon = new Pigeon(pigeonPort);
  // private Navx navx = new Navx(Navx.Port.I2C);
  // private DriveBase base = new DriveBase(controller, navx, DriveType.Tank);
  // private Elevator elevator = new Elevator(base, controller);
  // private Hatch hatch = new Hatch(controller, base);

  // private AutoMaster auto = new AutoMaster(base, navx, hatch);

  // private String m_autoSelected;
  private static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();
  // private Relay visionRelay = new Relay(0);

  @Override
  public void robotInit() {

    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    // auto.start();
    // visionRelay.set(Relay.Value.kForward);

    Hardware.canifier.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {
    /*
     * Scheduler.getInstance().run();
     * 
     * byte[] dataMessage = arduino.readSerialPort();
     * 
     * if (dataMessage.length == 2) { int dataByte0 = (int)
     * (Byte.toUnsignedInt(dataMessage[0])); int dataByte1 = (int)
     * (Byte.toUnsignedInt(dataMessage[1]));
     * 
     * switch (proximityState) {
     * 
     * case LookingForID:
     * 
     * System.out.println("DataByte0: " + dataByte0);
     * System.out.println("DataByte1 " + dataByte1);
     * 
     * if (dataByte0 == 0xff) { if (dataByte1 == 0x01 || dataByte1 == 0x2) {
     * proximityState = proximityEnum.LookingForData; } }
     * 
     * break;
     * 
     * case LookingForData:
     * 
     * System.out.println("Proximity Sensor " + leftDistance.distanceInches() +
     * "--------------------------"); proximityState = proximityEnum.LookingForID;
     * 
     * break;
     * 
     * case Error: default:
     * 
     * break;
     * 
     * }
     * 
     * }
     */

  }

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
    // base.teleopInit();
    // auto.start();
  }

  @Override
  public void autonomousPeriodic() {
    // auto.run();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopInit() {
    // base.teleopInit();

    

     
  for(ILoopable taskAnimateLEDStrip : Tasks.FullList){
  Schedulers.PeriodicTasks.add(taskAnimateLEDStrip);
    }}
    // pigeon.resetYaw();

  

  @Override
  public void teleopPeriodic() {
    
     serialDataHandler.readPort();
    //if (loopIndex++ == 5) {
     System.out.println(String.format("sensor1Data: %d ", serialDataHandler.getSensor1Data()));
     System.out.println(String.format("sensor2Data: %d ", serialDataHandler.getSensor2Data()));
     //}
    // base.TeleopMove();

      // elevator.teleopRaise();
      // elevator.raiseElevator();

    // pixy.read();
    // hatch.teleop();
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
    Schedulers.PeriodicTasks.process();

  }

  @Override
  public void testPeriodic() {
    //base.teleopInit();
    //base.rightFront.set(ControlMode.Position, 10000);
    //System.out.println("Counts: " + base.rightFront.getSensorPosition());
    //System.out.println("hello fam: " + base.rightFront.getSensorVelocity());
    // base.TestEncoders();
    //hatch.testPeriodic();
    // elevator.testLiftDriveEncoder();
  }

}