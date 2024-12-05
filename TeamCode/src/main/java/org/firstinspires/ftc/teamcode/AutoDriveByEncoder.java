package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder (Net)", group = "Robot")

public class AutoDriveByEncoder extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

        // Initialize all the hardware using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Set through each let of the path.
        // Note: Each tile is ~24 inches by ~ 24 inches
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Dive forward 2 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, 2, 2, 2, 2., 5);

        // Strafe left 50 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, -50, 50, 50,  -50, 5);
        // Pushes a preloaded sample into the net zone

        // Strafe right 20 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, 20, -20, -20, 20, 5);

        // Dive forward 52 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, 52, 52, 52, 52, 5);

        // Strafe left 8 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, -8, 8, 8, -8, 5);

        // Turn Clockwise 2 inch with a 5-second timeout
        robot.driveEncoder(robot.TURN_SPEED, 2, 2, -2, -2, 5);

        // Dive backward 48 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, -48, -48, -48, -48, 5);
        // pushes the inside specimen into the net

        // Dive forward 48 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, 48, 48, 48, 48, 5);

        // Strafe left 10 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, -10, 10, 10, -10, 5);

        // Dive backward 48 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, -48, -48, -48, -48, 5);
        // pushes the middle specimen into the net

        // Dive forward 48 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, 48, 48, 48, 48, 5);

        // Strafe left 10 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, -10, 10, 10, -10, 5);

        // Dive backward 44 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, -44, -44, -44, -44, 5);
        // pushes the outside specimen into the net

        // Strafe right 24 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, 24, -24, -24, 24, 5);

        // Dive forward 48 inches with a 5-second timeout
        robot.driveEncoder(robot.DRIVE_SPEED, 48, 48, 48, 48, 5);

        // Strafe right 16 inches with a 5-second timeout
        robot.driveEncoder(robot.STRAFE_SPEED, 16, -16, -16, 16, 5);
        // Parks in the accent zone


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pauses to display thr final telemetry message.
    }
}
