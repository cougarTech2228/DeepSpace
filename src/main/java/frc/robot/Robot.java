/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveBase.DriveType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public ProximitySensor distance = new ProximitySensor(I2C.Port.kOnboard);
  private Pigeon pigeon = new Pigeon(RobotMap.PIGEONIMU);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  
  private Arduino arduino = new Arduino();
  private DriverIF controller = new DriverIF();
  private Navx navx = new Navx(Navx.Port.I2C);
  private DriveBase base = new DriveBase(controller, navx, DriveType.Tank);
  private AutoMaster auto = new AutoMaster(base, navx);
  private Hatch hatch = new Hatch(controller, base);

  private String m_autoSelected; 
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    auto.start();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    base.teleopInit();
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
    base.TeleopMove();
    // pixy.read();
    hatch.teleop();
    //Scheduler.getInstance().run();
    //arduino.test();
    //pigeon.pigeonCheck();
    //System.out.println(navx.getYaw());

    //PixyData p = new PixyData();
    /*
    try {
      p = pixy.readPacket(1);
      if(p == null)
      p = new PixyData();
    } catch(Exception e) {
      e.printStackTrace();
    }
    System.out.println("X: " + p.X + "Y: " + p.Y + "Width: " + p.Width + "Height: " + p.Height);*/
    
    //pixy.read();
  }
  @Override
  public void testPeriodic() {
    base.TestEncoders();
  }
}