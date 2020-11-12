import  lejos.robotics.navigation.*;
import  lejos.hardware.motor.EV3LargeRegulatedMotor; 
import  lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import  lejos.robotics.chassis.Chassis; 
import  lejos.robotics.chassis.Wheel; 
import  lejos.robotics.chassis.WheeledChassis;
import  lejos.hardware.Button;
import lejos.hardware.device.*;
import lejos.hardware.port.*;
//import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.*;

public class MapTest{

   static  EV3LargeRegulatedMotor  LEFT_MOTOR  =  new  EV3LargeRegulatedMotor(MotorPort.B); 
   static  EV3LargeRegulatedMotor  RIGHT_MOTOR  =  new  EV3LargeRegulatedMotor(MotorPort.A);

   public static void main(String[] args){

      SensorModes uss_sensor = new EV3UltrasonicSensor(SensorPort.S4);
      SampleProvider distance_provider = uss_sensor.getMode("Distance");
      // Don't start until a button is pressed
      //Button.waitForAnyPress();

      /* setup the wheel diameter of left (and right) motor  in centimeters (2.0 cm)
       the offset number is the distance between the center of wheel 
       to the center of robot (half of track width)*/
      
      
      //TODO: rotate 180 and keep track of 
      int x = 29;
      int y = 29;
      int r = 270;
      int[][] map = new int[30][30];
      
      for (int i = 0; i < map.length; i++) {
          for (int j = 0; j < map[i].length; j++) {
              map[i][j] = 0;
          }
      }
      Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR , 3.17).offset(-7.62);
      Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR , 3.17).offset(7.62);

      // set up the  chassis  type:  Differential pilot
      Chassis chassis = new WheeledChassis(new  Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
      float[] ranges = new float[distance_provider.sampleSize()];
      
      MovePilot pilot = new MovePilot(chassis);

       float distanceValue = 0;
       	int index = 0;
        while(index<100){

          
        	index++;
            float [] sample = new float[distance_provider.sampleSize()];
            distance_provider.fetchSample(sample, 0);
            distanceValue = sample[0];
            System.out.println(distanceValue);
            if(distanceValue<.1){

                switch(r) {
                case 90:
                if(x >= map.length) {
                	pilot.rotate(180);
                }
                map[y][x+1] = 1;
                break;
                case 180:
                map[y+1][x] = 1;
               break;
                case 270:
                	 map[y][x-1]= 1;
                break;

                case 0:
                	 map[y-1][x] = 1;

                break;
            
                }
                pilot.rotate(-110);
                r+=90;
                if(r==360){
                r=0;
                }
            	}
            else {
            	 pilot.travel(5);

                 switch(r) {
                 case 90:
                 x++;
                 break;
                 case 180:
                  y++;
                break;
                 case 270:
                 x--;
          
                 break;

                 case 0:
                 y--;

                 break;
             
                 }
            }
           
            Move move = pilot.getMovement();
            System.out.println("X=" +x);
            System.out.println("Y=" +y);

        
        }
        
        for (int i = 0; i < map.length; i++) {
            for (int j = 0; j < map[i].length; j++) {
                System.out.print(map[i][j] +" , ");
                
            }
            System.out.println("");
        }
 

      }

      //drive the robot forward infinitly or untill commanded to stop


      //if you want the robot to travel a certain distance (100 cm in this case)
      //negative distance moves the robot in the oposite dirrection

      
      //if you want to turn 90 degrees


      // press the ESCAPE button to stop moving 

      }