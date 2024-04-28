
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class FullAutoPath extends SequentialCommandGroup {
  
  public FullAutoPath(Drivetrain drive) {

    addCommands(
      new PIDLine(0.9, drive),
      new PIDTurn(-90, drive),
      new PIDLine(1.77, drive),
      new PIDTurn(-180, drive),
      new PIDLine(1.77, drive),
      new PIDTurn(90, drive),
      new PIDLine(0.91, drive)
    );
  }
}
