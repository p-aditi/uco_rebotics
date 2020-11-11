import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.internal.ev3.EV3LCD;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
//import ev3.projects.library.*;

import java.util.HashMap;

import lejos.hardware.Battery;
import lejos.hardware.BrickFinder;

import lejos.hardware.LED;

import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;



import lejos.utility.Delay;
import lejos.hardware.device.*;

import lejos.hardware.port.*;
//import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;




public class MapTracker
{ 
    //private static final String int i  = null;
	// Motors
    static EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(MotorPort.A);
    static EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.B);
    //Touch Sensor Setup
    static EV3TouchSensor        touch = new EV3TouchSensor(SensorPort.S1);
    static SensorModes uss_sensor = new EV3UltrasonicSensor(SensorPort.S4);
    
    
    public static void main(String[] args)
    {
    

    	int[][] gridmap = new int[2][2];
    	
		SampleProvider distance_provider = uss_sensor.getMode("Distance");
		float[] ranges = new float[distance_provider.sampleSize()];
    
    	int n = 360;
    	float [] map = new float[n];
    	float [] angles = new float[n];
        int index=0;
		EV3GyroSensor gyro;
		gyro = new EV3GyroSensor(SensorPort.S3);
			
		SampleProvider provider = gyro.getAngleMode();
		
		float angle = 0;
		float [] array; 
		
        
       
        Button.LEDPattern(4);    // flash green led and 
        Sound.beepSequenceUp();  // make sound when ready.

        //System.out.println("Press any key to start");
        
        //Button.waitForAnyPress();
        
        motorA.setSpeed(20);
        motorB.setSpeed(20);
       
        motorA.forward();
        motorB.forward();
        int r = 0;
        int c = 0;
        if(motorA.getPosition() < -300 && motorB.getPosition() < -300) {
        	for(c = 0; c < gridmap[r].length; c++) {
        			gridmap[r][c] = 1;
        		}
        }
        else if(motorA.getPosition() > -300 || motorB.getPosition() >-300){
        	motorA.stop();
        	motorB.stop();
        	motorA.setSpeed(100);
        	Delay.msDelay(200);
        	motorA.stop();
        	motorA.setSpeed(40);
            motorB.setSpeed(40);
        	r += 1;
        	for(c = 0; c < gridmap[r].length; c++) {
    			gridmap[r][c] = 1;
    		}
        }
        
        
        // drive waiting for touch sensor or escape key to stop driving.
        SampleProvider sp = touch.getTouchMode();
        while (!isTouched(sp) && index < n)
        {
        	
        	
          distance_provider = uss_sensor.getMode("Distance");
          ranges = new float[distance_provider.sampleSize()];
          provider = gyro.getAngleMode();
          array = new float[provider.sampleSize()];
          provider.fetchSample(array, 0);
		  angle = (float)array[0];
          if(Math.abs(angle) % (float)(360/n) == 0){
            distance_provider.fetchSample(ranges, 0);
                map[index]= ranges[0]; 
                angles[index]=Math.abs(angle);
                index++;
          }
           
            LCD.clear(7);
            //LCD.print(7,  "value=%.3f", colorValue);
        
        }
        
        double x;
        double y;
        
//        for (int i=0;i<angles.length;i++) {
//        	
//        	x = Math.sin(angles[i])*map[i];
//        	y = Math.cos(angles[i])*map[i];
//            //System.out.println("X = " + x + " Y = "+ y);
//        }
        // stop motors with brakes on.
        motorA.stop();
        motorB.stop();

        // free up resources.
        motorA.close();
        motorB.close();
        touch.close();
        //color.close();
        gyro.close();
        Sound.beepSequence(); // we are done.
    }
    
    public static boolean isTouched( SampleProvider sp)
    {
       float [] sample = new float[sp.sampleSize()];

       sp.fetchSample(sample, 0);

       if (sample[0] == 0)
           return false;
       else
           return true;
    }
}