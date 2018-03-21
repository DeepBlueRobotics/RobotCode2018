package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.IntakeEjectInterface;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class OpenIntake extends InstantCommand {

	private IntakeEjectInterface intEj;

	public OpenIntake(IntakeEjectInterface intEj) {
		super();
		requires(Robot.intakeEject);
		this.intEj = intEj;
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		intEj.openIntake();
	}

}
