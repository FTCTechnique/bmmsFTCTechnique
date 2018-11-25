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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
/*
 * This is an example LinearOpMode that shows how to use
 * a REV Robotics Touch Sensor.
 *
 * It assumes that the touch sensor is configured with a name of "digitalTouch".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
//@TeleOp(name = "FirstAutoMission", group = "Sensor")
@Autonomous(name = "ColorAutoMission", group = "Sensor")
//@Disabled
public class ColorAutoMission extends LinearOpMode {
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
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    // private DcMotor armDrive = null;
    // private Servo armServo = null;



    @Override
    public void runOpMode() throws InterruptedException {
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


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
        // armDrive=hardwareMap.get(DcMotor.class,"elbow");
        //Initialize servos
        //  armServo = hardwareMap.get(Servo.class, "arm_servo");
        //Set servo position
        //armServo.setPosition(.8);

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

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (sensorColor.blue() < 70) {
                telemetry.addData("Sensor Color", "Is not Blue");
                DriveForward(-.2);

            }
            else {
                telemetry.addData("Sensor Color", "Is Blue");
            //    DriveForwardTime(.2,1000);
            //    TurnLeftTime(.4,2000);
            //    DriveForwardTime(-.2,1000);
                  stop();
            }

            telemetry.update();
        }
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



