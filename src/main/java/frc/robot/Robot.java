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
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.Relay;
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
  private static int pigeonPort = RobotMap.PIGEONIMU;
  private static Pigeon pigeon = new Pigeon(pigeonPort);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private DriverIF controller = new DriverIF();
  private DriveBase base = new DriveBase(controller, pigeon, DriveType.Tank);
  private SerialDataHandler serialDataHandler = new SerialDataHandler(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
  private Hatch hatch = new Hatch(controller, base);
  private Relay visionRelay = new Relay(0, Direction.kForward);
  private Elevator elevator = new Elevator(base, controller);
  private int count;

  // private static int pigeonPort = RobotMap.PIGEONIMU;
  // private static Pigeon pigeon = new Pigeon(pigeonPort);
  // private Navx navx = new Navx(Navx.Port.I2C);
    
  // private DriveBase base = new DriveBase(controller, navx, DriveType.Tank);
  // private Hatch hatch = new Hatch(controller, base);
  private AutoMaster auto = new AutoMaster(base, hatch);

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // private static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();

  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    auto.start();
  }

  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  @Override
  public void teleopInit() {
    hatch.teleopInit();
    elevator.teleopInit();
    visionRelay.set(Value.kOn);
  }

  @Override
  public void teleopPeriodic() {
    /*
    if(serialDataHandler.getSensor1Data() == -1 || serialDataHandler.getSensor2Data() == -1){
      System.out.println("Sensor Data old, == -1");
     }
     else{
      System.out.println(String.format("sensor1Data: %d ", serialDataHandler.getSensor1Data()));
      System.out.println(String.format("sensor2Data: %d ", serialDataHandler.getSensor2Data()));
     }*/

    
    if (controller.toggleLights()) {
      visionRelay.set(Relay.Value.kOn);
    } else {
      visionRelay.set(Relay.Value.kOff);
    }
    //System.out.println("pidgey: " + pigeon.getYaw());
    base.TeleopMove();
    //elevator.teleopRaise();
    //elevator.teleopPeriodic();
    hatch.teleop();
    // base.teleopInit();

    

         
  //for(ILoopable taskAnimateLEDStrip : Tasks.FullList){
  //Schedulers.PeriodicTasks.add(taskAnimateLEDStrip);
    }

  @Override
  public void testInit() {
    hatch.teleopInit();
    base.autoInit();
    
  }

  @Override
<<<<<<< HEAD
  public void testPeriodic() {
    // elevator.updateSwitches();
    elevator.putElevatorEncoders();
=======
  public void testPeriodic(){
    //base.teleopInit();
    //base.rightFront.set(ControlMode.Position, 10000);
    //System.out.println("Counts: " + base.rightFront.getSensorPosition());
    //System.out.println("hello fam: " + base.rightFront.getSensorVelocity());
    // base.TestEncoders();
    //hatch.testPeriodic();
    //elevator.updateSwitches();
    //  elevator.testLiftDriveEncoder();
    // for (ILoopable taskAnimateLEDStrip : Tasks.FullList) {
    //   Schedulers.PeriodicTasks.add(taskAnimateLEDStrip);
    // }
    // Schedulers.PeriodicTasks.process();
>>>>>>> 0defcc4d4219554c33b6b31fb03b09c4c45b8f59
  }

}