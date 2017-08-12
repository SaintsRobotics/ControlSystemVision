package org.usfirst.frc.team1899.robot;

import edu.wpi.first.wpilibj.CameraServer;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	RobotDrive testRobot;
	Joystick stick;
	int autoLoopCounter, teleopLoopCounter;
	ADXRS450_Gyro gyro;
	AnalogInput ai0;
	DigitalInput di0;
	//PWM PWM_Test;
	
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	myRobot = new RobotDrive(0,1);
    	testRobot = new RobotDrive(2,3);
    	stick = new Joystick(0);
    	gyro = new ADXRS450_Gyro();
    	ai0 = new AnalogInput(0);
    	di0 = new DigitalInput(0);
    	VisionSquareSensor vision = new VisionSquareSensor();
    	//PWM_Test = new PWM(2);
    	//CameraServer.getInstance().startAutomaticCapture();
       	gyro.calibrate();
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Blur", IMG_WIDTH, IMG_HEIGHT);
        Mat source = new Mat();
        Mat output = new Mat();
        cvSink.grabFrame(source);
//        Imgproc.cvtColor(source, output, Imgproc.COLOR_RGBA2BGR);
        outputStream.putFrame(source);
        
 
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	teleopLoopCounter = 0;
    	gyro.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    //    myRobot.arcadeDrive(stick);
    	double leftSpeed,rightSpeed;
    	
    	leftSpeed=stick.getRawAxis(1);
    	rightSpeed=stick.getRawAxis(5);
    	myRobot.tankDrive(leftSpeed,rightSpeed,true);
    	testRobot.tankDrive(leftSpeed,rightSpeed,true);
    	//PWM_Test.setSpeed(leftSpeed);
    	
    	SmartDashboard.putNumber("Left Speed", leftSpeed);
    	SmartDashboard.putNumber("Gyro Angle" , gyro.getAngle());
    	SmartDashboard.putNumber("Gyro Angle Graph" , gyro.getAngle());
    	SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
    	SmartDashboard.putNumber("Right Speed", rightSpeed);
    	SmartDashboard.putNumber("Pot (ai0)" , ai0.getVoltage());
    	SmartDashboard.putBoolean("di0", di0.get());
    	
    	//SmartDashboard.putNumber("Teleop Counter", teleopLoopCounter++);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
