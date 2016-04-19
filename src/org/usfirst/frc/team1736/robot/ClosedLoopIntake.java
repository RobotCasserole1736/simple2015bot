package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class ClosedLoopIntake extends PIDSubsystem {
	
	static double P = 0.0025;
	static double I = 0.0003;
	static double D = 0.0006;

	final double ENCODER_DEG_PER_TICK = 360.0/(120.0*4.0);
	
	final double RETRACT_DEGREES = 360;
	
	Victor intake_motor;
	Encoder intake_encoder;
	
	public volatile IntLncState present_state;
	public volatile IntLncState next_state;
	
	public ClosedLoopIntake() {
		super("ClosedLoopIntakePID", P, I, D); //we don't need to WPILIB feed forward. we do feed fowrard ourselfs cuz they were silly with their implementation.
    	intake_motor = new Victor(5);
    	intake_encoder = new Encoder(0,1);
    	intake_encoder.setReverseDirection(true);
		setOutputRange(-1,1); //Must not command the motor in reverse since the input speed taken as unsigned (negative motor commands cause instability)
		present_state = IntLncState.STOPPED_NO_BALL;
		next_state = IntLncState.STOPPED_NO_BALL;
		enable();
	}

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return intake_encoder.getRaw()*ENCODER_DEG_PER_TICK;
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		if(next_state == IntLncState.INTAKE){
			intake_motor.set(1);
			this.setSetpoint(intake_encoder.getRaw()*ENCODER_DEG_PER_TICK);
		}
		else if (next_state == IntLncState.STOPPED_NO_BALL){
			intake_motor.set(0);
			this.setSetpoint(intake_encoder.getRaw()*ENCODER_DEG_PER_TICK);
		}
		else if(next_state == IntLncState.RETRACT){
			if(present_state != IntLncState.RETRACT){
				this.setSetpoint(intake_encoder.getRaw()*ENCODER_DEG_PER_TICK + RETRACT_DEGREES);
			}
			intake_motor.set(-output); //invert motor direction
		}
			
		present_state = next_state;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
		//memes
		
	}

}
