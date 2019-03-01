/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.DriveBase.DriveType;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static Pigeon pigeon = new Pigeon(RobotMap.PIGEONIMU);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private DriverIF controller = new DriverIF();
  private DriveBase base = new DriveBase(controller, pigeon, DriveType.Tank);
  private SerialDataHandler serialDataHandler = new SerialDataHandler(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
  private Vision vision = new Vision();
  private Hatch hatch = new Hatch(controller, vision);
  private Elevator elevator = new Elevator(base, controller);

  private AutoMaster auto = new AutoMaster(base, hatch, vision, controller);
  //private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // private static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();

  @Override
  public void robotInit() {

    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("rightkF", 0);
    SmartDashboard.putNumber("leftkF", 0);
    SmartDashboard.putNumber("right kP", 0.01);
    SmartDashboard.putNumber("left kP", 0.01);
    SmartDashboard.putNumber("right kI", 0.001);
    SmartDashboard.putNumber("left kI", 0.001);
    SmartDashboard.putNumber("right Izone", 30);
    SmartDashboard.putNumber("left Izone", 30);
    SmartDashboard.putNumber("right kD", 10);
    SmartDashboard.putNumber("left kD", 10);
    SmartDashboard.putNumber("speed", 0.5);
    //visionRelay.set(Relay.Value.kForward);
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    // auto.start();
    // visionRelay.set(Relay.Value.kForward);

    // Hardware.canifier.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
    // base.teleopInit();
    // auto.start();
    auto.init();
  }

  @Override
  public void autonomousPeriodic() {
    //auto.run();
  }

  @Override
  public void teleopInit() {
    //base.teleopInit();
    vision.visionInit();
    auto.teleop();
    hatch.teleopInit();
    elevator.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println("pidgey: " + pigeon.getYaw());
    /*
    if(serialDataHandler.getSensor1Data() == -1 || serialDataHandler.getSensor2Data() == -1){
      System.out.println("Sensor Data old, == -1");
     }
     else{
      System.out.println(String.format("sensor1Data: %d ", serialDataHandler.getSensor1Data()));
      System.out.println(String.format("sensor2Data: %d ", serialDataHandler.getSensor2Data()));
     }*/

    auto.teleop();
    vision.setRelay(controller.toggleLights());
    //System.out.println("pidgey: " + pigeon.getYaw());
    base.TeleopMove();
    //elevator.teleopRaise();
    //elevator.teleopPeriodic();
    vision.teleop();
    hatch.teleop();
    
    // base.teleopInit();

    

    /*
    for(ILoopable taskAnimateLEDStrip : Tasks.FullList){
      Schedulers.PeriodicTasks.add(taskAnimateLEDStrip);
    }
    */
  }

  @Override
  public void testInit() {
    hatch.teleopInit();
    base.autoInit();
    
  }

  @Override
  public void testPeriodic() {

  }
}