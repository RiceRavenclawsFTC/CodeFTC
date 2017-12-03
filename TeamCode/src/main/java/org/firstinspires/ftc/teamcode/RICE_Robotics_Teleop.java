
/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIzAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

        package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.util.RobotLog;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Ananth V on 9/1/2017.
 */

@TeleOp(name="Bubbles TeleOp", group="Linear OpMode")
public class RICE_Robotics_Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRiceRavens robot           = new HardwareRiceRavens(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Use Custom Hardware

    double servo;
    double servo2;
    double armPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double armspeed;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.servo.setPosition(0.5);
        robot.servo2.setPosition(0.5);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // loop-run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = -gamepad1.left_stick_y * 0.7 - gamepad1.right_stick_x * 0.6;
            right = -gamepad1.left_stick_y * 0.7 + gamepad1.right_stick_x * 0.6;
            armspeed = -gamepad2.right_stick_y * 0.3;

            robot.motor1.setPower(left);
            robot.motor2.setPower(right);
            robot.motor3.setPower(armspeed);
            servo = robot.servo.getPosition();
            servo2 = robot.servo2.getPosition();
            if(gamepad2.y) {
                // move to 0 degrees.
                servo += 0.075;
                servo2 -= 0.075;
            } else if (gamepad2.x) {
                // move to 90 degrees.
                servo += 0.05;
            } else if (gamepad2.b) {
                // move to 180 degrees.
                servo2 -= 0.05;
            } else if (gamepad2.a) {
                // move to 180 degrees.
                servo -= 0.075;
                servo2 += 0.075;
            } else if (gamepad2.left_bumper) {
                // move to 180 degrees.
                servo = 0.5;
                servo2 = 0.5;
            }
            servo = Range.clip(servo, 0, 0.5);
            servo2 = Range.clip(servo2, 0.5, 1);
            robot.servo.setPosition(servo);
            robot.servo2.setPosition(servo2);
            telemetry.addData("Servo Position", robot.servo.getPosition());
            telemetry.addData("Servo Position2", robot.servo2.getPosition());
            telemetry.addData("Status", "Running");


            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            //Function in hardware class
            robot.waitForTick(40);
        }
    }
}