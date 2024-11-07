package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends Command {
    
    DriveTrain driveTrain;
    Joystick joy;
    
    public DriveCommand(DriveTrain driveTrain, Joystick joy) {
        this.driveTrain = driveTrain;
        this.joy = joy;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveTrain.Px = driveTrain.Deadzone(joy.getRawAxis(0));
        driveTrain.Py = driveTrain.Deadzone(-joy.getRawAxis(1));
        driveTrain.Px2 = driveTrain.Deadzone(-joy.getRawAxis(4));
        driveTrain.Py2 = driveTrain.Deadzone(joy.getRawAxis(5));

        driveTrain.pov = joy.getPOV();

        driveTrain.TriggerVel = joy.getRawAxis(3) - joy.getRawAxis(2);

        driveTrain.SpeedMode(
            joy.getRawButton(1), 
            joy.getRawButton(2), 
            joy.getRawButton(3)
            );

        if ((driveTrain.Px == 0 && driveTrain.Py == 0) && (driveTrain.Px2 != 0 || driveTrain.Py2 != 0)) {
            driveTrain.AxiSpeeds(driveTrain.Px2, driveTrain.Py2);
        } else {
            driveTrain.AxiSpeeds(driveTrain.Px, driveTrain.Py);
        }

        if (driveTrain.pov != -1) {
            driveTrain.POV();
        }

        driveTrain.setMotors();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.StopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
