package frc.robot;

import org.opencv.video.KalmanFilter;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.usfirst.frc.team3035;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions correspondin to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	private static final String kDefaultAuto = "DriveStraight";

	// Command autonomousCommand;
	private String m_autoSelected;
	SendableChooser<String> autoChooser = new SendableChooser<>();

	String gameData; // returns L or R in a string of 3 chars, in order corresponding to your
						// alliance
	AHRS ahrs;

	Spark LF, LB, RF, RB;
	Spark intakeMotor;
	Spark liftL, liftR;
	Spark arm;
	SpeedControllerGroup L,R;

	Compressor c = new Compressor(1);					//pass the PCM Node ID 
	boolean pressureSwitch = c.getPressureSwitchValue(); 
	//DoubleSolenoid armSolenoid_1;
	//DoubleSolenoid armSolenoid_2;
	DoubleSolenoid liftSolenoid_1;
	DoubleSolenoid liftSolenoid_2;
	DoubleSolenoid cSolenoid;
	/*
	Solenoid notes
	---------------
	exampleDouble = new DoubleSolenoid(1,2);				//(moduleNumber, forwardChannel, reverseChannel)
	exampleDouble = new DoubleSolenoid (1,1,2);				//

	exampleDouble.set(DoubleSolenoid.Value.kOff) 			//puts solenoid in neutral position
	exampleDouble.set(DoubleSolenoid.Value.kForward)		//forward channel enabled
	exampleDouble.set(DoubleSolenoid.Value.kReverse)		//reverse channel enabled
	**just noticed that this is from an archived article form 2014 so it might not be up to date**
	*/
	
	// Spark LF, LB;
	// Victor RF, RB, LF, LB;//, iL, iR;
	//Spark iL, iR;// , iRF, iRB;
	//Spark flip;

	Timer timer;
	PIDController turnController;
	private double rotateToAngleRate;

	Joystick player1 = new Joystick(1), player2 = new Joystick(2);

	static final double kP = 0.175;
	static final double kI = 0.01;
	static final double kD = 0.115;
	static final double kF = 0.00;


	//turnPID turner= new turnPID(kP,kI,kD);
	static final double kToleranceDegrees = 2.0f;

	static final double kTargetAngleDegrees = -90.0f;

	double target;
	/*
	 * Compressor compressor; DoubleSolenoid exSoloIn; DoubleSolenoid exSoloIn2;
	 */

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@SuppressWarnings("deprecation")
	@Override
	public void robotInit() {
		/*
		 * m_chooser.addDefault("Default Auto", kDefaultAuto);
		 * m_chooser.addObject("Auto (Start Left)", kLeftAuto);
		 * m_chooser.addObject("Auto (Start Center)", kCenterAuto);
		 * m_chooser.addObject("Auto (Start Right)", kRightAuto);
		 * SmartDashboard.putData("Auto choices", m_chooser);
		 */
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Default program", kDefaultAuto);

		SmartDashboard.putData("Auto", autoChooser);

		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

		camera.setResolution(320, 240);
		camera.setFPS(30);

		LF = new Spark(6);
		LB = new Spark(5);
		RF = new Spark(8);
		RB = new Spark(7);
		liftL = new Spark(1);
		liftR = new Spark(0);
		R = new SpeedControllerGroup(RF,RB);
		L = new SpeedControllerGroup(LF,LB);

		arm = new Spark(0);
		intakeMotor = new Spark(2);

		c.setClosedLoopControl(false);				//automatically will keep pressure ~120 psi when true
		//exampleSolenoid = new DoubleSolenoid(power number, forward, reverse)
		//armSolenoid_1 = new DoubleSolenoid(0, 0, 1);
		//armSolenoid_2 = new DoubleSolenoid(0, 2, 3);
		liftSolenoid_1 = new DoubleSolenoid(0, 7,6);
		liftSolenoid_2 = new DoubleSolenoid(0, 2,3);
		cSolenoid = new DoubleSolenoid(0, 0, 1);
		//armSolenoid_1.set(DoubleSolenoid.Value.kOff);	//initalize solenoid to neutral position
		//armSolenoid_2.set(DoubleSolenoid.Value.kOff);	//initalize solenoid to neutral position
		liftSolenoid_1.set(DoubleSolenoid.Value.kOff);	//initalize solenoid to neutral position
		liftSolenoid_2.set(DoubleSolenoid.Value.kOff);	//initalize solenoid to neutral position
		

		//	flip = new Spark(6);
		//	iL = new Spark(5);
		//	iR = new Spark(0);

		LF.enableDeadbandElimination(true);
		LB.enableDeadbandElimination(true);
		RF.enableDeadbandElimination(true);
		RB.enableDeadbandElimination(true);

		arm.enableDeadbandElimination(true);
		intakeMotor.enableDeadbandElimination(true);

		//flip.enableDeadbandElimination(true);
		//	iL.enableDeadbandElimination(true);
		//	iR.enableDeadbandElimination(true);
		// iL = new Victor(2);
		// iR = new Victor(3);
		/*
		 * iLF= new Spark(8); iLB = new Spark(7); iRF = new Spark(9); iRB = new
		 * Spark(6); lift = new Spark(5); ACTUAL
		 */
		timer = new Timer();
		/*
		 * compressor = new Compressor(0);
		 * 
		 * exSoloIn = new DoubleSolenoid(1, 0); exSoloIn2 = new DoubleSolenoid(2, 3);
		 * 
		 * compressor.start();
		 */

		gameData = DriverStation.getInstance().getGameSpecificMessage(); // to test, go onto
		// driver station software and enter game datal

		/*
		 * if (gameData.charAt(0) == 'L') // or 'R'; 1 for scale, 2 for opposing switch
		 * {
		 * 
		 * }
		 */

		try {
			/***********************************************************************
			 * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
			 * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(Port.kMXP);
		} 
		catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.disable();

		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */

	@Override
	public void autonomousInit() {

		m_autoSelected = autoChooser.getSelected();

		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		gameData = DriverStation.getInstance().getGameSpecificMessage(); // to test, go onto
		// driver station software and enter game datal

		/*
		 * if (gameData.charAt(0) == 'L') // or 'R'; 1 for scale, 2 for opposing switch
		 * { leftSwitch = true; } else if (gameData.charAt(0) == 'R') { rightSwitch =
		 * true; }
		 */
		turnController.disable();
		timer.reset();
		timer.start();
		ahrs.zeroYaw();

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		// Scheduler.getInstance().run();

		if (isAutonomous() && isEnabled()) {
			switch (m_autoSelected) {

			case kDefaultAuto:
			default:
				
				if (timer.get() < 1.3) {
					driveStraight(0.5, 1, 0);
				}
				else if (timer.get() > 1.4 && timer.get() <= 3){
					//turn 45 degrees
					turn(45, 0.2);
					
					//turn -45 degrees
					//turn(-45, 0.2); right side of field
				}
				else if (timer.get() > 3 && timer.get() < 15) {
					RB.set(0);
					RF.set(0);
					LF.set(0);
					LB.set(0);
					timer.stop();
				}
				
				break;
			}
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	boolean driveToggle = false; // false means Tank Drive, true means Arcade Drive.
	//boolean planeToggle = false; // false means up on the joystick equals flipping upward.
	boolean driveLastPass = false;

	@Override
	public void teleopInit() {
		// TODO Auto-generated method stub
		//turner.turn(50);
		super.teleopInit();
	}
	@Override
	public void teleopPeriodic() {
		//System.out.println(turner.get());
		double left = (player2.getRawAxis(1) * -1); // *-1 to inverse left side | left_stick_y
		double right = (player2.getRawAxis(1) * 1); // right_stick_y

		double turn = (player2.getRawAxis(1) * -.5); // left stick y
		//double power = (player2.getRawAxis(4) * .5); // right stick 

		double lift = (player1.getRawAxis(5) * .7);	//arm lift
		double intaker = (player1.getRawAxis(3) * .1);
		double outtake = (player1.getRawAxis(2) * -.3);

		/*if(pressureSwitch == true){			//if pressure is low
			c.start();						//start compressor
		}
		else if(pressureSwitch == false){	//if pressure is not low
			c.stop();						//stop compressor
		}
		else{
			c.stop();
		}*/

		liftL.set(lift);
		liftR.set(-lift);

		RF.set(right);
		RB.set(right);
		LF.set(left);
		LB.set(left);

		RF.set(turn);
		RB.set(turn);
		LF.set(-turn);
		LB.set(-turn);

		intakeMotor.set(outtake);
		intakeMotor.set(intaker);


		


		// double turbo = (player1.getRawAxis(2) * 1);
		/*
		if (player1.getRawButton(6)) {
			iL.set(.1);
			iR.set(.1);
		}
		if (intake > 0) {
			iL.set(intake * -1);
			iR.set(intake * -1);
		}

		else if (outtake > 0) {
			iL.set(outtake);
			iR.set(outtake);
		}

		else if (adjust > 0) {
			iL.set(adjust);
			// iR.set(adjust);
		}

		else {
			iL.set(0);
			iR.set(0);
		}
		flip.set(lift);
		*/

		//the following conditionals change drive mode (driveToggle)
		
		if (player2.getRawButton(1)) {
			/*
			 * While this button is held down, rotate to target angle. Since a Tank drive
			 * system cannot move forward simultaneously while rotating, all joystick input
			 * is ignored until this button is released.
			 */
			if (!turnController.isEnabled()) {
				turnController.setSetpoint(kTargetAngleDegrees);
				rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
				turnController.enable();
			}
			double leftStickValue = rotateToAngleRate;
			double rightStickValue = rotateToAngleRate;

			LF.set(leftStickValue * 0.5);
			LB.set(leftStickValue * 0.5);

			RF.set(rightStickValue * 0.5);
			RB.set(rightStickValue * 0.5);

			if (Math.abs(ahrs.getAngle() - kTargetAngleDegrees) <= 3) {
				LF.set(leftStickValue * 0.1);
				LB.set(leftStickValue * 0.1);

				RF.set(rightStickValue * 0.1);
				RB.set(rightStickValue * 0.1);
			}

			else {
				LF.set(leftStickValue * 0.5);
				LB.set(leftStickValue * 0.5);

				RF.set(rightStickValue * 0.5);
				RB.set(rightStickValue * 0.5);
			}

		} else if (player2.getRawButton(3)) {
			/*
			 * "Zero" the yaw (whatever direction the sensor is pointing now will become the
			 * new "Zero" degrees.
			 */
			ahrs.zeroYaw();
		} else if (player2.getRawButton(2)) {
		}

		//ball in/outake controls
		
		
		//arm controls
		arm.set(lift);

		//pneumatic controls
		//-------------------
		//deploy hatch panel
		/*if (player1.getRawButton(4)){
			//hatch panel arm solenoids
			armSolenoid_1.set(DoubleSolenoid.Value.kForward);
			armSolenoid_2.set(DoubleSolenoid.Value.kForward);
		}
		//retract hatch panel
		else if (player1.getRawButton(1)){
			armSolenoid_1.set(DoubleSolenoid.Value.kReverse);
			armSolenoid_2.set(DoubleSolenoid.Value.kReverse);
		}*/
		if(player1.getRawButton(1)){
			cSolenoid.set(DoubleSolenoid.Value.kOff);
		}

		//deploy lift robot
		if (player1.getRawButton(3)){
			//lift solenoids
			//exampleDouble.set(DoubleSolenoid.Value.kOff/kForward/kReverse);
			//
			liftSolenoid_1.set(DoubleSolenoid.Value.kForward);
			liftSolenoid_2.set(DoubleSolenoid.Value.kForward);
		}
		//retract lift
		else if (player1.getRawButton(2)){
			liftSolenoid_1.set(DoubleSolenoid.Value.kReverse);
			liftSolenoid_2.set(DoubleSolenoid.Value.kReverse);			
		}

	}

	/**
	 * This function is called periodically during test mode.
	 */

	@Override
	public void testPeriodic() {

		driveStraight(0.5, 1, 0);

	}
	/* Standard tank drive, no driver assistance. */

	/*
	 * LF.set(left); LB.set(left);
	 * 
	 * RF.set(right); RB.set(right);
	 */

	 /*public void intake() {
		iL.set(.75);
		iR.set(.75);
		// iLB.set(.5);
		// iLF.set(.5);
		// iRB.set(.5);
		// iRF.set(-.5);
	}

	public void outtake() {
		iL.set(-.75);
		iR.set(-.75);
		// iLB.set(-.6);
		/// iLF.set(-.4);
		// iRB.set(.6);
		// iRF.set(.4);
	}

	public void midtake() {
		// iLB.set(.3);
		// iLF.set(.5);
		// iRB.set(-.3);
		// iRF.set(-.5);
	}

	public void mistake() {
		// iLB.set(.3);
		// iLF.set(-.5);
		// iRB.set(-.3);
		// iRF.set(.5);

	}

	public void stopIntake() {
		// iLB.set(0);
		// iLF.set(0);
		// iRB.set(0);
		// iRF.set(0);
		iL.set(0);
		iR.set(0);
	}
 */
	public void setGyro() {
		ahrs.zeroYaw();

	}

	public void driveStraight1(double speed) {
		double leftSpeed;
		double rightSpeed;

		double currentPos = ahrs.getYaw();
		leftSpeed = 0.5 - (currentPos - target) / 100;
		rightSpeed = (0.5 + (currentPos - target) / 100) * -1;

		/*
		 * // Left - negative - current pos = -1, target = 0 leftSpeed = 0.5; rightSpeed
		 * = -0.5; /* LF.set((.5 + rotateToAngleRate) * 0.5); LB.set((.5 +
		 * rotateToAngleRate) * 0.5); RF.set((-.5 - rotateToAngleRate) * 0.5);
		 * RB.set((-.5 - rotateToAngleRate) * 0.5);
		 */

		LB.set(leftSpeed);
		LF.set(leftSpeed);
		RF.set(rightSpeed);
		RB.set(rightSpeed);

	}

	public void driveStraight(double speed, int direction, int target) {
		
		double leftSpeed = 0;
		double rightSpeed = 0;

		double currentPos = ahrs.getYaw();
		//target = 0;

		if (direction == 1) {
			leftSpeed = 0.55 - (currentPos - target) / 100;
			rightSpeed = (0.5 + (currentPos - target) / 100); // inverse
		}

		else if (direction == -1) {
			leftSpeed = (0.5 + (currentPos - target) / 100);
			rightSpeed = (0.5 - (currentPos - target) / 100);
		}

		/*
		 * // Left - negative - current pos = -1, target = 0 leftSpeed = 0.5; rightSpeed
		 * = -0.5; /* LF.set((.5 + rotateToAngleRate) * 0.5); LB.set((.5 +
		 * rotateToAngleRate) * 0.5); RF.set((-.5 - rotateToAngleRate) * 0.5);
		 * RB.set((-.5 - rotateToAngleRate) * 0.5);
		 */

		if (direction == 1) {
			LB.set(leftSpeed);
			LF.set(leftSpeed);
			RF.set(-rightSpeed);
			RB.set(-rightSpeed);
		}

		else if (direction == -1) {
			LB.set(leftSpeed * -1);
			LF.set(leftSpeed * -1);
			RF.set(-rightSpeed);
			RB.set(-rightSpeed);
		}
	}

	public void turnLeft() {
		turn(-90);
	}

	public void turnRight() {
		turn(90);
	}

	public void turn(double angle) {
		turnController.setSetpoint(angle);

		double turnSpeed = rotateToAngleRate;
		
		turnController.enable();
		if (angle < 0) { // turning left
			LF.set(turnSpeed * 0.5);
			LB.set(turnSpeed * 0.5);

			RF.set(turnSpeed * 0.5);
			RB.set(turnSpeed * 0.5);

			if (Math.abs(ahrs.getAngle() - angle) <= 3) {
				LF.set(turnSpeed * 0.1);
				LB.set(turnSpeed * 0.1);

				RF.set(turnSpeed * 0.1);
				RB.set(turnSpeed * 0.1);
			}

			else {
				LF.set(turnSpeed * 0.5);
				LB.set(turnSpeed * 0.5);

				RF.set(turnSpeed * 0.5);
				RB.set(turnSpeed * 0.5);
			}
		} else if (angle > 0) { // turning right
			LF.set(turnSpeed * 0.5);
			LB.set(turnSpeed * 0.5);

			RF.set(turnSpeed * 0.5);
			RB.set(turnSpeed * 0.5);

			if (Math.abs(ahrs.getAngle() - angle) <= 3) {
				LF.set(turnSpeed * 0.1);
				LB.set(turnSpeed * 0.1);

				RF.set(turnSpeed * 0.1);
				RB.set(turnSpeed * 0.1);
			}

			else {
				LF.set(turnSpeed * 0.5);
				LB.set(turnSpeed * 0.5);

				RF.set(turnSpeed * 0.5);
				RB.set(turnSpeed * 0.5);
			}
		}

	}

	public void turn(double angle, double rotateToAngleRate) {
		turnController.setSetpoint(angle);

		double turnSpeed = rotateToAngleRate;
		if (angle < 0) { // turning left
			LF.set(turnSpeed * 0.5);
			LB.set(turnSpeed * 0.5);

			RF.set(turnSpeed * 0.5);
			RB.set(turnSpeed * 0.5);

			if (Math.abs(ahrs.getAngle() - angle) <= 3) {
				LF.set(turnSpeed * 0.1);
				LB.set(turnSpeed * 0.1);

				RF.set(turnSpeed * 0.1);
				RB.set(turnSpeed * 0.1);
			}

			else {
				LF.set(turnSpeed * 0.5);
				LB.set(turnSpeed * 0.5);

				RF.set(turnSpeed * 0.5);
				RB.set(turnSpeed * 0.5);
			}
		} else if (angle > 0) { // turning right
			LF.set(turnSpeed * -0.5);
			LB.set(turnSpeed * -0.5);

			RF.set(turnSpeed * -0.5);
			RB.set(turnSpeed * -0.5);

			if (Math.abs(ahrs.getAngle() - angle) <= 3) {
				LF.set(turnSpeed * -0.1);
				LB.set(turnSpeed * -0.1);

				RF.set(turnSpeed * -0.1);
				RB.set(turnSpeed * -0.1);
			}

			else {
				LF.set(turnSpeed * -0.5);
				LB.set(turnSpeed * -0.5);

				RF.set(turnSpeed * -0.5);
				RB.set(turnSpeed * -0.5);
			}
		}

	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}

}