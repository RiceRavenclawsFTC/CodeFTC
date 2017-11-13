package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ananth V on 9/1/2017.
 */

public class HardwareRiceRavens {
    DcMotor leftMotor_upper = null;
    DcMotor rightMotor_upper  = null;

    HardwareMap map = null;
    Servo servo; Servo servo2;
    private DcMotor.RunMode initialMode = null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareRiceRavens(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public void init(HardwareMap aMap) {
        map = aMap; //sets null map, creates local copy

        leftMotor_upper = map.dcMotor.get("leftMotor_upper"); //sets leftmotor from name of motor on phone
        rightMotor_upper = map.dcMotor.get("rightMotor_upper");
        leftMotor_upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor_upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor_upper.setMode(initialMode);
        rightMotor_upper.setMode(initialMode);

        leftMotor_upper.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_upper.setDirection(DcMotorSimple.Direction.FORWARD);

        servo = map.get(Servo.class, "left");
        servo2 = map.get(Servo.class, "right");

        stopRobot();
    }

    public void stopRobot() {
        leftMotor_upper.setPower(0);
        rightMotor_upper.setPower(0);
    }


    public void waitForTick(long periodMs) {
        long  remaining = periodMs - (long) period.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
