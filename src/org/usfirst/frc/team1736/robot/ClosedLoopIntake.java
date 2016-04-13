package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class ClosedLoopIntake extends PIDSubsystem {
	
	static double P = 0.005;
	static double I = 0;
	static double D = 0;

	static final double ENCODER_TICKS_PER_DEG = 360/2048;
	
	static final double RETRACT_DEGREES = 480;
	
	Victor intake_motor;
	Encoder intake_encoder;
	
	public volatile IntLncState present_state;
	public volatile IntLncState next_state;
	
	public ClosedLoopIntake() {
		super("ClosedLoopIntakePID", P, I, D); //we don't need to WPILIB feed forward. we do feed fowrard ourselfs cuz they were silly with their implementation.
    	intake_motor = new Victor(5);
    	intake_encoder = new Encoder(0,1);
    	intake_encoder.setDistancePerPulse(ENCODER_TICKS_PER_DEG);
		setOutputRange(-1,1); //Must not command the motor in reverse since the input speed taken as unsigned (negative motor commands cause instability)
		present_state = IntLncState.STOPPED_NO_BALL;
		next_state = IntLncState.STOPPED_NO_BALL;
		enable();
	}

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		intake_encoder.getDistance();
		return 0;
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		if(next_state == IntLncState.INTAKE){
			intake_motor.set(-1);
			this.setSetpoint(intake_encoder.getDistance());
		}
		else if (next_state == IntLncState.STOPPED_NO_BALL){
			intake_motor.set(0);
			this.setSetpoint(intake_encoder.getDistance());
		}
		else{
			if(present_state != IntLncState.RETRACT && next_state == IntLncState.RETRACT){
				this.getPIDController().reset();
				this.setSetpoint(intake_encoder.getDistance() - RETRACT_DEGREES);
			}
			intake_motor.set(output);
		}
			
		present_state = next_state;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
		//memes
		
	}

}
