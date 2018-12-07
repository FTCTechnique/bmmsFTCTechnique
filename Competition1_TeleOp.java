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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Competition1_TeleOp", group="Linear Opmode")
//@Disabled
public class Competition1_TeleOp extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo handServo = null;
    private DcMotor shoulder = null;
    private DcMotor elbow =  null;
    private Servo teamMarker = null;
    private Servo touchServo = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBack = hardwareMap.get (DcMotor.class, "left_back");
        rightBack = hardwareMap.get (DcMotor.class, "right_back");
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        teamMarker=hardwareMap.get(Servo.class, "team_marker");
        touchServo = hardwareMap.get(Servo.class, "touch_servo");

        //Initialize the DC Motor in the arm
        shoulder = hardwareMap.get(DcMotor.class,"shoulder");
        elbow = hardwareMap.get(DcMotor.class,"elbow");

        //Set servo position
        handServo.setPosition(.8);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


           //Gamepad 1 for Base Driver

            // Uses left stick in "arcade" mode to go forward, backward, turn right and left
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;

            // Use A down to put Team Marker down and Y to lift it up
            if (gamepad1.a)
                    teamMarker.setPosition(.2);
            else if (gamepad1.y)
                    teamMarker.setPosition(.4);

            // Use X to put touch sensor up and B to put touch sensor down
            if (gamepad1.x)
                    touchServo.setPosition(.2);
            else if (gamepad1.b)
                    touchServo.setPosition(.4);

            // Gamepad 2 for Arm Driver

            //Use A to put arm in travel mode
            //if (gamepad2.a)
              //  shoulder.setPower(-.3);
                //elbow.setPower(-.3);

            //Use B to close the hand
            if (gamepad2.b)
                handServo.setPosition(0);

            //Use X to open the hand
            else if (gamepad2.x)
                handServo.setPosition(1);

            //Use left bumper to move shoulder up
            if (gamepad2.left_bumper)
                shoulder.setPower(-.3);

            //Use left trigger to move shoulder down
            else if (gamepad2.left_trigger !=0)
                    shoulder.setPower(.3);
            else
                    shoulder.setPower (0);

            //Use right bumper to move elbow up
            if (gamepad2.right_bumper)
                elbow.setPower(.3);
            //Use right trigger to put elbow down
            else if (gamepad2.right_trigger !=0)
                elbow.setPower(.3);
            else
                elbow.setPower(0);



            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
