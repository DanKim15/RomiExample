package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoTestPath extends Command {
  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVolts);
  private final Drivetrain m_drive;
  private long startTime;
  private final long endTime = 2000;
  public AutoTestPath(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(ff.calculate(0.51), ff.calculate(0.51));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime > endTime;
  }
}
