// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This is a demo program showing the use of Tallon SRX magic motion to control a motor.
 */
public class Robot extends TimedRobot {


  //private final Joystick m_stick = new Joystick(0);
  private final XboxController c_stick = new XboxController(1);
   
  private static final int demo_Motor_Channel = 4;
  WPI_VictorSPX lift_Motor = new WPI_VictorSPX(5);
  
	WPI_TalonSRX demo_Motor = new WPI_TalonSRX(demo_Motor_Channel);
  
  /** Convert desired distance (linear travel) to number of encoder edges.
  The gear ratio is the gearing between the encoder and the output shaft.
  Note: 4096 are the number of edges detected per revolution of the encoder.
  */
  
private double Distance_per_Rev = 1;
private double gear_Ratio = 3;

private double distance_demo_Motor (double travel_Distance_inches) {
  double a = ((travel_Distance_inches * Distance_per_Rev) * gear_Ratio) * 4096;;
  return a;
}
 
  // Set crusing speed at the encoder.

/** Convert RPM to number of counts per 100m sec where each revolution = 4096 

cs_demo_Motor_RPM is the target RPM (at the encoder)
4096 is the EPR (Edges per Rotation) of the encoder
600 is the number of time steps per minute.

Talon SRX uses a velocity unit of Sensor value per 100ms. 
There are 600 of these 100ms time steps per minute (10 per second * 60 seconds per minute). 
So the 600 is what’s used to convert “per minute” to “per 100ms”.
  */
private double cs_demo_Motor_RPM = 2000;
private double cs_demo_Motor = (cs_demo_Motor_RPM / 600) * 4096;

  /** Set Acceleration speed at the encoder.

  This sets the time it takes the motor to reach crusing speed.
   */
  private double ac_demo_Motor_Seconds = 1.5;
  private double ac_demo_Motor = cs_demo_Motor / ac_demo_Motor_Seconds;


  @Override
  public void robotInit() {

// Configure the encoder to relative.
  demo_Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  demo_Motor.setSensorPhase(false);
    
    /** demo_motor PID Feedback - Start
   
    We are controlling the motor by setting its velocity
    Every 100 ms the motor controller reads the number of edges it sees and compares this to the set point.
    It then increases or decreases the power to the motor to get it to run at the correct speed.
    PID values tells the controller how to how to make this adjustment.
    PID values can be "tuned" using the Phoenix tuner software.
    */
    
      /* Config the Velocity closed loop gains (PID) in slot0 */

    demo_Motor.config_kP(0, .08, 0);
    demo_Motor.config_kI(0, 0, 0);
    demo_Motor.config_kD(0, 1, 0);
  /* Shooter PID Feedback - End*/
  
  /* Motion Magic Configurations */
  demo_Motor.configMotionAcceleration(ac_demo_Motor, 30);
  demo_Motor.configMotionCruiseVelocity(cs_demo_Motor,30);
   
  /** Set the Sensor position to zero.
   * You can also use the limit switch sensor to "home" the zero position.
   * Example
   *       if (demo_Motor.getSensorCollection().isRevLimitSwitchClosed()){
        demo_Motor.setSelectedSensorPosition(0);
      }
   * */
  // Important - The zero point will be where we position the linear arm at the start of the match.
 demo_Motor.setSelectedSensorPosition(0);
  
  }

  @Override
  public void teleopPeriodic() {
 

    if (c_stick.getAButtonPressed()){
   
    
      demo_Motor.set (ControlMode.MotionMagic, distance_demo_Motor(4));
    
     }
     else if (c_stick.getBButtonPressed()){
     
  
       demo_Motor.set (ControlMode.MotionMagic, distance_demo_Motor(.5));
      
      }


}

  }

