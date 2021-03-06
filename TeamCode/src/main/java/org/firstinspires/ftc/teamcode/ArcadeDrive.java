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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Code for Sky Stone FTC 2019
 *
 * Team 14561
 */

@TeleOp(name="ArcadeDrive", group="UltimateGoal")
public class ArcadeDrive extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain drivetrain;
    private Arm arm;
    private FlyWheel flywheel;
    private Sweeper sweeper;
    private WobbleArm wobbleArm;
    private PusherMotor pusher;
    //private HeightSensor heightSensor;
    private RingSensor ringSensor;
    boolean powerShotGo = false;
    private PowerShotAuton powerShotAuton;
    private int stage = 60;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        wobbleArm = new WobbleArm(hardwareMap, telemetry);
        drivetrain = new DriveTrain(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        flywheel = new FlyWheel(hardwareMap,telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        pusher = new PusherMotor(hardwareMap, telemetry);
        //heightSensor = new HeightSensor(hardwareMap, telemetry);
        ringSensor = new RingSensor(hardwareMap, telemetry);
        powerShotAuton = new PowerShotAuton(arm,
                 drivetrain,
                 flywheel,
                 pusher,
                 sweeper,
                 wobbleArm,
                 telemetry,
                 runtime);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad1.y){
            powerShotGo = true;
        }
        
        if(powerShotGo){
            arm.manual(0.0, true, false, RobotMap.POT_POWER_POSITION);
            double driveTrainEncoder = drivetrain.rightEncoder.getCurrentPosition();
            stage = powerShotAuton.powerShoot(stage, driveTrainEncoder);
            telemetry.addData("Stage", stage);
            if(stage == 90 || gamepad1.x){
                powerShotGo = false;
            }
        } else {
            drivetrain.arcadeDrive(gamepad1);
            arm.manual(gamepad2);
            flywheel.manual(gamepad2);
            sweeper.buttonServo(gamepad2);
            pusher.manual(gamepad2);
            //heightSensor.broadcastHeight();
            ringSensor.broadcastColor();
            wobbleArm.manual(gamepad1);
            stage = 60;
        }


        // Show the elapsed game time.
        if (RobotMap.DISPLAY_TIME) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
        telemetry.update();
    }

    //Controls Arm to prevent colliding with ground
    /*
    public void collisionAvoidance(double goal){
        double totalArmHeight = RobotMap.ELEVATOR_ENCODER_CONSTANT * elevator.getEncoder() + RobotMap.ELEVATOR_MIN_HEIGHT;
        if(RobotMap.ARM_LENGTH * Math.cos(RobotMap.ARM_ENCODER_CONSTANT * goal) >= totalArmHeight - RobotMap.ARM_GROUND_CLEARANCE){
            int armEncoderValue = (int)(Math.acos(totalArmHeight / RobotMap.ARM_LENGTH) / RobotMap.ELEVATOR_ENCODER_CONSTANT);
            arm.setAutonGoal(armEncoderValue);
        }
    }
     */


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
