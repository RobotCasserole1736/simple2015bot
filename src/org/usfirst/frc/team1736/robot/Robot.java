
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	Victor DTLF_motor;
	Victor DTRF_motor;
	Victor DTLB_motor;
	Victor DTRB_motor;
	Victor slide_motor;
	
	ClosedLoopIntake intake;

	
	Xbox360Controller joy1;
	
	public final double SLIDE_CORR_GAIN = 0.3;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	DTLF_motor = new Victor(0);
    	DTRF_motor = new Victor(3);
    	DTLB_motor = new Victor(1);
    	DTRB_motor = new Victor(2);
    	slide_motor = new Victor(4);
    	intake = new ClosedLoopIntake();
    	
    	joy1 = new Xbox360Controller(0);
    }
    

    public void autonomousInit() {

    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	double x = signedSquared(joy1.LStick_X());
    	double y = signedSquared(-joy1.LStick_Y());
    	double z = signedSquared(joy1.RStick_X());
    	
    	double frontRightMotorValue = limitMotorCmd(y + x * SLIDE_CORR_GAIN - z);
    	double frontLeftMotorValue  = limitMotorCmd(y - x * SLIDE_CORR_GAIN + z);
    	double backRightMotorValue  = limitMotorCmd(y + x * SLIDE_CORR_GAIN - z);
    	double backLeftMotorValue   = limitMotorCmd(y - x * SLIDE_CORR_GAIN + z);   	
    	double slideMotorValue      = limitMotorCmd(x);
    	
    	DTRF_motor.set(-frontRightMotorValue); //invert because opposite side of robot.
    	DTLF_motor.set(frontLeftMotorValue);
    	DTRB_motor.set(-backRightMotorValue);
    	DTLB_motor.set(backLeftMotorValue);
		slide_motor.set(slideMotorValue);
		
		if(joy1.LB())
			intake.next_state = IntLncState.INTAKE;
		else if(joy1.B())
			intake.next_state = IntLncState.RETRACT;
		else
			intake.next_state = IntLncState.STOPPED_NO_BALL;
		
		SmartDashboard.putNumber("IntakeSetpoint", intake.getSetpoint());
		SmartDashboard.putNumber("IntakeActualAngle", intake.intake_encoder.getRaw() * intake.ENCODER_DEG_PER_TICK);
		SmartDashboard.putString("IntakeState", intake.present_state.toString());
		System.out.println(intake.intake_encoder.getRaw() * intake.ENCODER_DEG_PER_TICK);
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    /*
     * Utility math functions
     */
    private double signedSquared(double in){
    	if(in > 0)
    		return in*in;
    	else
    		return -1*in*in;
    }
    
    private double limitMotorCmd(double in){
    	if(in > 1)
    		return 1;
    	else if(in < -1)
    		return -1;
    	else
    		return in;
    }
    
}
