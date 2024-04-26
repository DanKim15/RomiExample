package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceTrap extends TrapezoidProfileCommand {
    public DriveDistanceTrap(double meters, Drivetrain drive) {
        super(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(DriveConstants.maxVelocity, DriveConstants.maxAccel)
            ),
            setpointState -> drive.ffDrive(setpointState.velocity, setpointState.velocity),
            () -> new TrapezoidProfile.State(meters, 0),
            TrapezoidProfile.State::new,
            drive);
        drive.resetEncoders();

        
    }
}