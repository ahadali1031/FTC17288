

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

        package org.firstinspires.ftc.teamcode.Robot;

        import android.app.Activity;
        import android.graphics.Color;
        import android.view.View;


        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

        import java.util.Locale;

        /*
         * This is an example LinearOpMode that shows how to use
         * the REV Robotics Color-Distance Sensor.
         *
         * It assumes the sensor is configured with the name "sensor_color_distance".
         *
         * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
         * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
         */

        // Comment this out to add to the opmode list
        public class ColorTest  {

                /**
                 * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
                 * It has an IR proximity sensor which is used to calculate distance and an RGB color sensor.
                 * <p>
                 * There will be some variation in the values measured depending on whether you are using a
                 * V3 color sensor versus the older V2 and V1 sensors, as the V3 is based around a different chip.
                 * <p>
                 * For V1/V2, the light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
                 * or closer will display the same value for distance/light detected.
                 * <p>
                 * For V3, the distance sensor as configured can handle distances between 0.25" (~0.6cm) and 6" (~15cm).
                 * Any target closer than 0.25" will dislay as 0.25" and any target farther than 6" will display as 6".
                 * <p>
                 * Note that the distance sensor function of both chips is built around an IR proximity sensor, which is
                 * sensitive to ambient light and the reflectivity of the surface against which you are measuring. If
                 * very accurate distance is required you should consider calibrating the raw optical values read from the
                 * chip to your exact situation.
                 * <p>
                 * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
                 * you can treat the sensor as two separate sensors that share the same name in your op mode.
                 * <p>
                 * In this example, we represent the detected color by a hue, saturation, and value color
                 * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
                 * color of the screen to match the detected color.
                 * <p>
                 * In this example, we  also use the distance sensor to display the distance
                 * to the target object.
                 */
                ColorSensor sensorColor;
                DistanceSensor sensorDistance;
                LinearOpMode opMode;
                Telemetry telemetry;

                final double SCALE_FACTOR = 255;

                public ColorTest(Telemetry teleme, LinearOpMode tempOpMode) {
                        telemetry = teleme;
                        opMode = tempOpMode;
                }


                public void init(HardwareMap hardwareMap) {
                        sensorColor = hardwareMap.get(ColorSensor.class, "cSensor");
                }



                // sometimes it helps to multiply the raw RGB values with a scale factor
                // to amplify/attentuate the measured values.



                // loop and read the RGB and distance data.
                // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
                public boolean parking() {
                        // convert the RGB values to HSV values.
                        // multiply by the SCALE_FACTOR.
                        // then cast it back to int (SCALE_FACTOR is a double)
//                        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                                (int) (sensorColor.green() * SCALE_FACTOR),
//                                (int) (sensorColor.blue() * SCALE_FACTOR);

                        // send the info back to driver station using telemetry function.
//                        telemetry.addData("Distance (cm)",
//                                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//                        telemetry.addData("Alpha", sensorColor.alpha());
//                        telemetry.addData("Red  ", sensorColor.red());
//                        telemetry.addData("Green", sensorColor.green());
//                        telemetry.addData("Blue ", sensorColor.blue());

                        // change the background color to match the color detected by the RGB sensor.
                        // pass a reference to the hue, saturation, and value array as an argument
                        // to the HSVToColor method.


//                        telemetry.update();
                        if ((sensorColor.red() > 200) && (sensorColor.green() > 200) && (sensorColor.blue() > 200)){
                                return true;
                        }
                        else{
                                return false;
                        }

                        // Set the panel back to the default color


                }

                public boolean alignment(){
                        if ((sensorColor.red() > 200) && sensorColor.green() < 75 && sensorColor.blue() < 75){
                                return true;
                        }
                        else
                                return false;
                }
        }



