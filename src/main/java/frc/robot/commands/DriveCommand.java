package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends Command {
    
    DriveTrain driveTrain;
    Joystick joy;
    
    double Px, Py, Px2, Py2;
    
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
        Px = driveTrain.Deadzone(joy.getRawAxis(0));
        Py = -driveTrain.Deadzone(joy.getRawAxis(1));
        Px2 = -driveTrain.Deadzone(joy.getRawAxis(4));
        Py2 = driveTrain.Deadzone(joy.getRawAxis(5));

        driveTrain.TriggerVel = joy.getRawAxis(3) - joy.getRawAxis(2);

        driveTrain.SpeedMode(
            joy.getRawButton(1), 
            joy.getRawButton(2), 
            joy.getRawButton(3)
            );

        if ((Px == 0 && Py == 0) && (Px2 != 0 || Py2 != 0)) {
            driveTrain.AxiSpeeds(Px2, Py2);
        } else {
            driveTrain.AxiSpeeds(Px, Py);
        }

        if (joy.getPOV() != -1) {
            driveTrain.POV(joy.getPOV());
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
