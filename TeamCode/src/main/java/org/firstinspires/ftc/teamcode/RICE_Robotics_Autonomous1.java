
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


/**
 * Created by Ananth V on 9/1/2017.
 */

@Autonomous(name="RICE_Robotics_Autonomous", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class RICE_Robotics_Autonomous1 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor1 = null;
    DcMotor motor2  = null;

    HardwareRiceRavens robot  = new HardwareRiceRavens(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    public void drive(double ld, double rd, long time){
        robot.motor1.setPower(ld);
        robot.motor2.setPower(rd);
        sleep(time);
    }

    public void shoot(){
        // launcher.setPower(-1); //If we have a launcher servo thingy
        // sleep(700);
        // launcher.setPower(0);
        // sleep(500);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        robot.init(hardwareMap);
        if (robot.ColorSensor.blue() > robot.ColorSensor.red()) {
            telemetry.addData("Color", "Blue");
        }
        else if (robot.ColorSensor.red() > robot.ColorSensor.blue()) {
            telemetry.addData("Color", "Red");
        }
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            drive(0.5, 0.5, 1500);
            robot.stopRobot();
        }
    }
}