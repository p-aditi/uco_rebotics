/*
*	Richu Mathew
*/
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Obstacle_detection {
	static EV3UltrasonicSensor uss = new EV3UltrasonicSensor(SensorPort.S4);
	static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		Button.LEDPattern(4);    // flash green led and
        Sound.beepSequenceUp();    // make sound when ready.
		
		SampleProvider sp = uss.getDistanceMode();
		int distance = 0;
		
		leftMotor.setSpeed(500);
		rightMotor.setSpeed(500);
		
		leftMotor.forward();
		rightMotor.forward();
		
		
		while(distance <= 30) {
			sp = uss.getListenMode();
			int sampleSize = sp.sampleSize();
			float[] samples = new float[sampleSize];
			sp.fetchSample(samples, 0);
			
			 Delay.msDelay(1000); // Delay for 1 sec
			
			sp = uss.getDistanceMode();
			sampleSize = sp.sampleSize();
			samples = new float[sampleSize];
			sp.fetchSample(samples, 0);
			
			distance = (int) samples[0];
			System.out.print("Distance = {}" + distance);
			
		}
		
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.close();
		rightMotor.close();
		
		Sound.beepSequence();

	}

}
