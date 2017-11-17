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
    DcMotor motor1 = null; //left motor
    DcMotor motor2  = null; // right motor
    DcMotor motor3 = null; //test motor for now
    DcMotor motor4  = null; //test motor for now

    HardwareMap map = null;
    Servo servo; Servo servo2;
    private DcMotor.RunMode initialMode = null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareRiceRavens(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public void init(HardwareMap aMap) {
        map = aMap; //sets null map, creates local copy

        motor1 = map.dcMotor.get("motor1"); //sets leftmotor from name of motor on phone
        motor2 = map.dcMotor.get("motor2");
        motor3 = map.dcMotor.get("motor3"); //sets leftmotor from name of motor on phone
        motor4 = map.dcMotor.get("motor4");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(initialMode);
        motor2.setMode(initialMode);
        motor3.setMode(initialMode);
        motor4.setMode(initialMode);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);

        servo = map.get(Servo.class, "left");
        servo2 = map.get(Servo.class, "right");

        stopRobot();
    }

    public void stopRobot() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
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
