/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This is an example LinearOpMode that shows how to use
 * a REV Robotics Touch Sensor.
 *
 * It assumes that the touch sensor is configured with a name of "digitalTouch".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */

@Autonomous(name = "CombinedAutoMission", group = "Sensor")
//@Disabled
public class CombinedAutoMission extends LinearOpMode {
    /**
     * The REV Robotics Touch Sensor
     * is treated as a digital channel.  It is HIGH if the button is unpressed.
     * It pulls LOW if the button is pressed.
     *
     * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
     * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
     * The lower (first) pin stays unconnected.*
     */

    DigitalChannel digitalTouch;  // Hardware Device Object// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor armDrive = null;
    private Servo handServo = null;
    private Servo teamMarker = null;
    private Servo touchServo = null;


    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBack = hardwareMap.get (DcMotor.class, "left_back");
        rightBack = hardwareMap.get (DcMotor.class, "right_back");
        // pulleyMotorUp = hardwareMap.get(DcMotor.class, "pulley_up");
        // pulleyMotorDown = hardwareMap.get(DcMotor.class,"pulley_down" );
        //Initialize the DC Motor in the arm
        armDrive=hardwareMap.get(DcMotor.class,"elbow");
        //Initialize servos
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        teamMarker = hardwareMap.get(Servo.class, "team_marker");
        touchServo = hardwareMap.get(Servo.class, "touch_servo");
        //Set servo position
        handServo.setPosition(.8);
        teamMarker.setPosition(1);
        touchServo.setPosition(1);

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        // armDrive.setDirection(DcMotor.Direction.REVERSE);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the touch sensor.

        //Lower Robot from Landing
        //MoveArmUpTime(.2, 1000);
        //handServo.setPosition(0);
        //sleep(200);
        //MoveArmDownTime (.2, 1000);


        while (opModeIsActive()) {
            // Drive forward until touch sensor is pressed
            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                DriveForwardTime(.2,1000); //CELINE
                DriveForward(.1); //CELINE

            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                DriveBackwardTime(.2,1000);
                TurnLeftTime(.4,2000); //CELINE
                // DriveForward(.2);
                //  markerServo.setPosition(0);
                // DriveForwardTime(.2,1000);
                // markerServo.setPosition(1);
                // DriveBackwardTime(.2,5000);
                // stop();
                break;
            }

            //wait();



            telemetry.update();
        }

        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                DriveForward(.2);

            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
                DriveBackwardTime(.2, 200);
                teamMarker.setPosition(.05); //CELINE
                DriveBackwardTime(.2,1000);
                teamMarker.setPosition(1);
                DriveBackwardTime(.7,3000); //CELINE
                stop();
                break;

            }

        }

    }


    private void MoveArmDownTime(double power, long time) throws InterruptedException {
        armDrive.setPower(-power);
        Thread.sleep(time);
    }

    private void MoveArmDown(double power) {
        armDrive.setPower(-power);
    }

    private void MoveArmUpTime(double power, long time) throws InterruptedException {
        armDrive.setPower(power);
        Thread.sleep(time);
    }

    private void MoveArmUp(double power) {
        armDrive.setPower(power);
    }

    private void DriveBackwardTime(double power, long time) throws InterruptedException {
        DriveBackward(power);
        Thread.sleep(time);
    }

    private void DriveBackward(double power) {
        DriveForward(-power);
    }

    private void DriveForward(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
    private void DriveForwardTime(double power,long time) throws InterruptedException {
        DriveForward(power);
        Thread.sleep(time);
    }
    private void TurnLeft(double power)  {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }
    private void TurnLeftTime(double power,long time) throws InterruptedException {
        TurnLeft(power);
        Thread.sleep(time);
    }
}



