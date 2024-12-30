package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Neutral Samples (Net)", group = "Robot")

public class AutoDriveByEncoder_NeutralNet extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){

        // Initialize all the hardware using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();

        // Set through each let of the path.
        // Note: Each tile is ~24 inches by ~ 24 inches
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        robot.driveAxial(robot.DRIVE_SPEED, 24, 0.0);
        robot.driveLateral(robot.STRAFE_SPEED, 24, 90);
        robot.turnToHeading(robot.TURN_SPEED, -180);
        robot.holdHeading(robot.TURN_SPEED, -180, 0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pauses to display thr final telemetry message.
    }
}
