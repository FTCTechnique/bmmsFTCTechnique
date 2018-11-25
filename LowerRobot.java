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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains team Technique's first autonomous program. An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This  particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="LowerRobot", group="Linear Opmode")
//@Disabled
public class LowerRobot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
   // private DcMotor rightDrive = null;
   // private DcMotor armDrive = null;
    private Servo armServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
       // rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        //Initialize the DC Motor in the arm
       // armDrive=hardwareMap.get(DcMotor.class,"elbow");
        //Initialize servos
        armServo = hardwareMap.get(Servo.class, "arm_servo");
        //Set servo position
        armServo.setPosition(0);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //armDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Make robot go


        DriveBackwardTime(-.2,500);
        MoveServo();
        DriveForwardTime(-.2,4000);
        StopDriving();
        //TurnLeftTime(.2,2000);
        //TurnRightTime(.2,2000);
        //MoveArmTime(.1,2000);
        //DriveForwardTime(.5,4000);
        //DriveBackwardTime(.5,4000);
        //TurnLeftTime(.5,2000);
        //TurnRightTime(.5,2000);
        //StopDriving();



        runtime.reset();

// Methods for basic robot movements.

    }

    private void MoveServo() {armServo.setPosition(.5);
    }

  //  private void MoveArm (double power) {
    //    armDrive.setPower(power);
  //  }
   // private void MoveArmTime(double power, long time) throws InterruptedException {
    //    MoveArm(power);
  //  }

    private void StopDriving() {
        DriveForward(0);
    }

   // private void TurnRight(double power) {
    //    TurnLeft(-power);

  //  }
   // private void TurnRightTime(double power,long time) throws InterruptedException {
     //   TurnRight(power);

    //}
   // private void TurnLeft(double power)  {
     //   leftDrive.setPower(-power);
     //   rightDrive.setPower(power);
    //}
    //private void TurnLeftTime(double power,long time) throws InterruptedException {
      //  TurnLeft(power);
        //Thread.sleep(time);
  //  }
    private void DriveBackward(double power) {
        leftDrive.setPower(-power);
   //     rightDrive.setPower(-power);
    }
    private void DriveBackwardTime(double power,long time) throws InterruptedException {
        DriveBackward(power);
        Thread.sleep(time);
    }
    private void DriveForward(double power) {
        leftDrive.setPower(power);
   //     rightDrive.setPower(power);
    }
    private void DriveForwardTime(double power,long time) throws InterruptedException {
        DriveForward(power);
        Thread.sleep(time);
    }
}
