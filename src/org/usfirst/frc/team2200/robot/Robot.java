package org.usfirst.frc.team2200.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;
import com.ctre.*;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	RobotDrive myRobot;
	CANTalon talon_left[];
	CANTalon talon_right[];
	Joystick stick = new Joystick(0);
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser<String> chooser = new SendableChooser<>();
	private final Object imgLock = new Object();
	double count1, count2;
	double hue[], sat[], lum[];
	DigitalOutput LED = new DigitalOutput(4);
	UsbCamera camera;
	Thread camera_background;
	double leftX, rightX, centerX;
	
	
	public Robot() {
		
	}

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
		GripPipeline gp = new GripPipeline();
		talon_left = new CANTalon[2];
		talon_right = new CANTalon[2];
		talon_left[0] = new CANTalon(1);
		talon_left[1] = new CANTalon(2);
		talon_right[0] = new CANTalon(3);
		talon_right[1] = new CANTalon(4);
		myRobot = new RobotDrive(talon_left[0], talon_left[1], talon_right[0], talon_right[1]);
		myRobot.setExpiration(0.1);
		
		gp.setHSL(20.0, 75.0, 44.0, 131.0, 160.0, 255.0);
		count1 = 0.0;
		count2 = 0.0;
		hue = new double[2];
		sat = new double[2];
		lum = new double[2];
		
		SmartDashboard.putNumber("Hue min", 30.0);
		SmartDashboard.putNumber("Hue max", 75.0);
		
		SmartDashboard.putNumber("Sat min", 130.0);
		SmartDashboard.putNumber("Sat max", 250.0);

		SmartDashboard.putNumber("Lum min", 160.0);
		SmartDashboard.putNumber("Lum max", 255.0);

        camera_background = new Thread(() -> {
            camera = CameraServer.getInstance().startAutomaticCapture();           
            camera.setResolution(640, 480);
            double count[] = new double[2];
            
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                gp.process1(source);
                //gp.process(source);
                outputStream.putFrame(gp.hslThresholdOutput());
                gp.process2();
                
                count[0] = gp.filterContoursOutput().size();
                count[1] = gp.findContoursOutput().size();
                //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                if (!gp.filterContoursOutput().isEmpty() && count[0] >= 2) {
                    Rect r = Imgproc.boundingRect(gp.filterContoursOutput().get(0));
                    Rect r1 = Imgproc.boundingRect(gp.filterContoursOutput().get(1));
                    
    	            synchronized (imgLock) {
    	            	leftX = r.x + (r.width / 2);
    	            	rightX = r1.x + (r1.width / 2);
    	            	centerX = (leftX + rightX) / 2;
    	            }
                    
                }
                
	            synchronized (imgLock) {
	                count1 = count[0];
	                count2 = count[1];
	            	gp.setHSL(hue[0], hue[1], sat[0], sat[1], lum[0], lum[1]);
	            }
                
            }
        });		
		
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomous() {
		/*
		String autoSelected = chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		switch (autoSelected) {
		case customAuto:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 1.0); // spin at half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		case defaultAuto:
		default:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		}
		*/
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	@Override
	public void operatorControl() {
		double count[] = new double[2];
		camera_background.start();
		
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			double leftX, rightX, centerX;
			//myRobot.arcadeDrive(stick); // drive with arcade style (use right
										// stick)

        	synchronized (imgLock) {
    			hue[0] = SmartDashboard.getNumber("Hue min",0.0);
    			hue[1] = SmartDashboard.getNumber("Hue max",0.0);
    			sat[0] = SmartDashboard.getNumber("Sat min",0.0);
    			sat[1] = SmartDashboard.getNumber("Sat max",0.0);
    			lum[0] = SmartDashboard.getNumber("Lum min",0.0);
    			lum[1] = SmartDashboard.getNumber("Lum max",0.0);
        		
        		count[0] = count1;
        		count[1] = count2;
	            
	            leftX = this.leftX;
	            rightX = this.rightX;
	            centerX = this.centerX;
        	}

			SmartDashboard.putNumber("count 1",count[0]);
			SmartDashboard.putNumber("count 2",count[1]);
			SmartDashboard.putNumber("left", leftX);
			SmartDashboard.putNumber("right", rightX);
			SmartDashboard.putNumber("center", centerX);
			
			if (stick.getRawButton(1))
				LED.set(true);
			if (stick.getRawButton(2))
				LED.set(false);
			if (stick.getRawButton(3)) {
				camera.setWhiteBalanceHoldCurrent();
				camera.setExposureHoldCurrent();
			}
			if (stick.getRawButton(4)) {
				camera.setWhiteBalanceAuto();
				camera.setExposureAuto();
			}
			
			if (centerX > 330.0) {
				myRobot.tankDrive(-0.5, 0.5);
			}
			else if (centerX < 310.0) {
				myRobot.tankDrive(0.5, -0.5);
			}
			else {
				myRobot.tankDrive(0.0, 0.0);
			}
			
			
			Timer.delay(0.005); // wait for a motor update time
		}
		LED.set(false);
		
		camera_background.interrupt();
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}
}
