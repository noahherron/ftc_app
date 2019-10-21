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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.lang.Math;

@Autonomous(name="Final Auto Modular")

public class finalAutoModular extends LinearOpMode
{

    // HARDWARE INITIALIZATION

        // Hardware configuration

        private DcMotor frontLeft = null;
        private DcMotor frontRight = null;
        private DcMotor backLeft = null;
        private DcMotor backRight = null;

        private BNO055IMU gyroscope = null;

        // Hardware initialization method

        public void initializeHardware()
        {

            // Motors

                // Initialize hardware maps

                frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
                frontRight = hardwareMap.get(DcMotor.class, "frontRight");
                backLeft = hardwareMap.get(DcMotor.class, "backLeft");
                backRight = hardwareMap.get(DcMotor.class, "backRight");

                gyroscope = hardwareMap.get(BNO055IMU.class, "gyroscope");

                // Set motor directions

                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.REVERSE);

                // Set motor run mode

                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Set zero power behavior

                frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Gyroscope

                // Gyroscope parameters

                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

                parameters.mode = BNO055IMU.SensorMode.IMU;
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.loggingEnabled = false;

                // Parameter initialization

                gyroscope.initialize(parameters);

        }

    // MOTOR INFO AND DISTANCE CALCULATION

        // Motor TPR, gear reduction, and wheel diameter information

        static final double ticksPerRevolution  = 1120;
        static final double driveGearReduction  = 27.0;
        static final double wheelDiameterInches = 3.93701;

        // Calculates ticks per inch based on gear reduction and wheel diameter

        static final double ticksPerInch = ( ticksPerRevolution * driveGearReduction ) / ( wheelDiameterInches * Math.PI );

        // Used to easily provide a inches distance to encoders

        public static int distance(double distanceInches)
        {

            // Calculate encoder ticks for the given distance

            double distanceTicks = distanceInches * ticksPerInch;


            // Return the distance in ticks

            return (int)distanceTicks;

        }

    // GYROSCOPE ORIENTATION INFORMATION

        Orientation angles = gyroscope.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

    // DRIVE AND TURN METHODS

        // Drive method

        public void drive(String direction, double targetDistance, double power)
        {
            if (direction == "forward")
            {
               // Reset encoders

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

               // Set target position

                frontLeft.setTargetPosition(distance(targetDistance));
                frontRight.setTargetPosition(distance(targetDistance));
                backLeft.setTargetPosition(distance(targetDistance));
                backRight.setTargetPosition(distance(targetDistance));

                // Run to position

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set drive power

                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);

                while ( frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() )
                {
                    // Wait until target position is reached
                }

                // Stop motors and revert back to normal

                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);

                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else if (direction == "backward")
            {
                // Reset encoders

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Set target position

                frontLeft.setTargetPosition(-distance(targetDistance));
                frontRight.setTargetPosition(-distance(targetDistance));
                backLeft.setTargetPosition(-distance(targetDistance));
                backRight.setTargetPosition(-distance(targetDistance));

                // Run to position

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set drive power

                frontLeft.setPower(-power);
                frontRight.setPower(-power);
                backLeft.setPower(-power);
                backRight.setPower(-power);

                while ( frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() )
                {
                    // Wait until target position is reached
                }

                // Stop motors and revert back to normal

                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);

                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            else if (direction == "left")
            {
                // Reset encoders

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Set target position

                frontLeft.setTargetPosition(distance(-targetDistance));
                frontRight.setTargetPosition(distance(targetDistance));
                backLeft.setTargetPosition(distance(targetDistance));
                backRight.setTargetPosition(distance(-targetDistance));

                // Run to position

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set drive power

                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(-power);

                while ( frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() )
                {
                    // Wait until target position is reached
                }

                // Stop motors and revert back to normal

                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);

                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            else if (direction == "right")
            {
                // Reset encoders

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Set target position

                frontLeft.setTargetPosition(distance(targetDistance));
                frontRight.setTargetPosition(distance(-targetDistance));
                backLeft.setTargetPosition(distance(-targetDistance));
                backRight.setTargetPosition(distance(targetDistance));

                // Run to position

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set drive power

                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(-power);
                backRight.setPower(power);

                while ( frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() )
                {
                    // Wait until target position is reached
                }

                // Stop motors and revert back to normal

                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);

                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            }
        }

        // Turn method

        public void turn(String direction, double targetRotation, double power)
        {

            if (direction == "left")
            {

                // Turn until target rotation is reached

                while (angles.firstAngle != targetRotation)
                {

                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                }

                // Stop motors and reset gyroscope

                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);

            }
            else if (direction == "right")
            {

                // Turn until target rotation is reached

                while (angles.firstAngle != -targetRotation)
                {

                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);

                }

                // Stop motors and reset gyroscope

                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);

            }
        }

    // OBJECT HANDLER



    @Override
    public void runOpMode()
    {

        // Wait for start

        initializeHardware();
        waitForStart();

        // Follow all directions

        drive("forward", 36, 1.0);
        turn("left", 90, 1.0);
        drive("left", 24, 1.0);

    }
}
