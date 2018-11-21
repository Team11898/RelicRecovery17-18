package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@TeleOp public class RaceWirelessConnectionCarConnectionOpModeConnection extends OpMode {

    public DcMotor rightFront;  // this is a standard thing that is needed
    public DcMotor leftFront;   // it basically sets up the program
    public DcMotor rightBack;   // its for wiring, like so u know how many motors are being used
    public DcMotor leftBack;   // and like how many servos
    public DcMotor slider;      // just a routine, no code to learn here :)
    public DcMotor intakeone;
    public DcMotor intaketwo; 
    public Servo servoup;
    public Servo servotwo;
    public Servo servothree;
    public Servo servofour;
    
    public DcMotor slider2;
    public Servo servocolor;
    public Servo servoball;

    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor odsSensor;  // Hardware Device Object


    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightFront");  // now we are assigning a name to each motor/servo
        leftFront = hardwareMap.dcMotor.get("leftFront");   // also nothing special here
        rightBack = hardwareMap.dcMotor.get("rightBack");   // 
        leftBack = hardwareMap.dcMotor.get("leftBack");
       
        slider = hardwareMap.dcMotor.get("slider");
        slider2 = hardwareMap.dcMotor.get("slider2");

        intakeone = hardwareMap.dcMotor.get("intakeone");
        intaketwo = hardwareMap.dcMotor.get("intaketwo");
        
        servoup = hardwareMap.servo.get("servoup");
        servotwo = hardwareMap.servo.get("servotwo");
        servocolor = hardwareMap.servo.get("servocolor");
        servothree = hardwareMap.servo.get("servothree");
        servofour = hardwareMap.servo.get("servofour");
        servoball = hardwareMap.servo.get("servoball");
        
        intakeone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //Slow down intake wheel one to a stop
        intaketwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Slow down intake wheel two to a
        
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");

        /*
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // RUN_USING_ENCODER makes  run @ same speeds
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intaketwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
    }
    
    @Override
    public void start(){
    servocolor.setPosition(.5);
    servoball.setPosition(.2);

    }

/*
    public void delay(double secs) {
        try {
            Thread.sleep((long) secs * 1000);    // function for time, can use "delay()" 
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
*/
    @Override
    public void loop() {
    // right stick y : forward and back
    // right stick x : turning left and right
    // triggers : strafing left and right
    // left stick x and y: diagonals
    
    telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
    //telemetry.addData("raw optical", rangeSensor.rawOptical());
    //telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
    telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
    
    telemetry.addData("Raw",    odsSensor.getRawLightDetected());
    telemetry.addData("Normal", odsSensor.getLightDetected());
    telemetry.update();

    
    
    leftBack.setPower(.4*(-gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(-gamepad1.left_stick_x));
    leftFront.setPower(.4*(-gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(gamepad1.left_stick_x));
    rightBack.setPower(.4*(gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(-gamepad1.left_stick_x));
    rightFront.setPower(.4*(gamepad1.right_stick_y)+ .6*gamepad1.right_stick_x + .7*(gamepad1.left_stick_x));
    
    //diagonal quadrants I and III
    /*
    if((gamepad1.left_stick_x>0)&&(gamepad1.left_stick_y>0) || (gamepad1.left_stick_x<0)&&(gamepad1.left_stick_y<0)){
        leftFront.setPower(.7*(gamepad1.left_stick_y));
        rightBack.setPower(.7*(-gamepad1.left_stick_y));
    }
    
    //diagonal quadrants II and IV
    if((gamepad1.left_stick_x<0)&&(gamepad1.left_stick_y>0) || (gamepad1.left_stick_x>0)&&(gamepad1.left_stick_y<0)){
        leftBack.setPower(.7*(gamepad1.left_stick_y));
        rightFront.setPower(.7*(-gamepad1.left_stick_y));
    }
*/
    

    
    

    //Servo
    
        if (gamepad2.b){ // SERVO LEVEL
            servoup.setPosition(.2);
            servotwo.setPosition(.8);
            servothree.setPosition(.8);
            servofour.setPosition(.2);
            
            
        }
        
        if (gamepad2.y){ // SERVO GO UP
            servoup.setPosition(.8);
            servotwo.setPosition(.2);
            servothree.setPosition(.2);
            servofour.setPosition(.8);
            
            
        }
        
        if(gamepad2.a){ // SERVO GO DOWN
            servoup.setPosition(0);
            servotwo.setPosition(1);
            servothree.setPosition(1);
            servofour.setPosition(0);
            
            
        }
        if (gamepad1.a){//centers Servo
            servocolor.setPosition(0.45);
            
        }    
            
        if(gamepad1.dpad_down){//servo go right
            servocolor.setPosition(0.3);
            
        }
        if(gamepad1.dpad_up){//servo go left 
            servocolor.setPosition(.6);
        }
        if(gamepad1.dpad_right){ //color servo go up
            servoball.setPosition(.95);
        }
        if(gamepad1.dpad_left){ //color servo go down
            servoball.setPosition(.2);
        }
        
    /*
        if (gamepad2.b){
            servocolor.setPosition(0.2);
        }
        
        if (gamepad2.a){
            servocolor.setPosition(.95);
        }
    
    */
    // DRAWER SLIDES 

        slider.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        
        slider2.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        
      
        
      
    //Collection  
        intakeone.setPower(gamepad2.right_stick_y*-.9);
        intaketwo.setPower(gamepad2.right_stick_y*.9);
        
     

    }


    @Override
    public void stop () {
    }
}



