package org.usfirst.frc.team199.Robot2018.commands;

import org.usfirst.frc.team199.Robot2018.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class EjectToExchange extends CommandGroup {

    public EjectToExchange() {
        addSequential(new LiftToPosition(Robot.lift, "GROUND"));
        addSequential(new OuttakeCube());
    }
}
