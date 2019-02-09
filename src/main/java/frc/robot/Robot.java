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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  
  private SerialDataHandler serialDataHandler = new SerialDataHandler();
  private DriverIF controller = new DriverIF();
  private Navx navx = new Navx(Navx.Port.I2C);
  private DriveBase base = new DriveBase(controller, navx, DriveType.Tank);
  private Hatch hatch = new Hatch(controller, base);
  private AutoMaster auto = new AutoMaster(base, navx, hatch);

  private String m_autoSelected; 
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private int loopIndex = 0;

  @Override
  public void robotInit() {
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    auto.start();
  }

  @Override
  public void robotPeriodic() {
    //Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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
    //base.teleopInit();
  }
  
  @Override
  public void teleopPeriodic() {
    serialDataHandler.readPort();
    if (loopIndex++ == 50) {
      System.out.println(serialDataHandler.getSensor1Data());
      System.out.println(serialDataHandler.getSensor2Data());
      loopIndex = 0;
   } 
  }

  @Override
  public void testPeriodic() {
  }
}