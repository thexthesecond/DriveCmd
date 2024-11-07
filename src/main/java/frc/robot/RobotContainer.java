package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  public DriveTrain driveTrain = new DriveTrain();
  public Joystick joy = new Joystick(Ports.kJoyPort);
  public Trigger driveTrigger;

  private final DriveCommand driveCommand;

  public RobotContainer() {
    driveCommand = new DriveCommand(driveTrain, joy);
    configureBindings();
  }

  public void configureBindings() {
    driveTrain.setDefaultCommand(driveCommand);
  }
}
