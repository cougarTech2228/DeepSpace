/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.DriveBase.DriveType;
import frc.robot.LEDUtilities.*;
import frc.robot.LEDs.Lights;

import com.ctre.phoenix.ILoopable;

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
  // private SerialDataHandler serialDataHandler = new SerialDataHandler(9600,
  // SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
  private Vision vision = new Vision();
  private Hatch hatch = new Hatch(controller, vision);
  private Elevator elevator = new Elevator(base, controller);

  private AutoMaster auto = new AutoMaster(base, hatch, vision, controller);
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //private static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();
  private Lights leds = new Lights();

  @Override
  public void robotInit() {
    Hardware.canifier.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
    leds.loop();
  }

  @Override
  public void autonomousInit() {
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    // for(ILoopable taskAnimateLEDStrip : Tasks.FullList){
    //   Schedulers.PeriodicTasks.add(taskAnimateLEDStrip);
    // }
    // auto.init();
    // base.teleopInit();
    // vision.visionInit();
    // hatch.teleopInit();
    elevator.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    // auto.teleop();
    // vision.setRelay(controller.toggleLights());
    // base.TeleopMove();
    elevator.teleopPeriodic();
    // vision.teleop();
    // hatch.teleop();
    System.out.println(pigeon.getPitch());
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