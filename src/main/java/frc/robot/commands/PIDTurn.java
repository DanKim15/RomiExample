// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class PIDTurn extends Command {

  private final Drivetrain m_drive;
  private double targetAngle;

  private final PIDController pid = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

  public PIDTurn(double angle, Drivetrain drive) {
    m_drive = drive; 
    targetAngle = angle;
    addRequirements(drive);
    pid.setTolerance(1.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setTargetAngle(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = MathUtil.clamp(pid.calculate(m_drive.getGyroAngleZ(), m_drive.getTargetAngle()), -0.5, 0.5);
    if (targetAngle < 0) {
      m_drive.tankDrive(input, -input);
    }
    else {
      m_drive.tankDrive(-input, input);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
