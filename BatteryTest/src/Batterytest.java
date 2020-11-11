import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.*;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class Batterytest {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Battery robot_battery = new Battery();
		float total_distance = 0;
		System.out.println("Starting motors on A and B");
        final EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(MotorPort.A);
        final EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.B);
        float motorA_position = 0;
        float motorB_position = 0;
        
        motorA.setSpeed(500);
        motorB.setSpeed(500);
        motorA.stop();
        motorB.stop();
        
        total_distance += motorA_position - motorA.getPosition();
        motorA_position = motorA.getPosition();
        motorB_position = motorB.getPosition();
        
        System.out.println("Forward");
        motorA.forward();
        motorB.forward();
        System.out.println("Large Motor A is moving: " +  motorA.isMoving() + " at speed {}" + motorA.getSpeed());
        System.out.println("Large Motor B is moving: " +  motorB.isMoving() + " at speed {}" + motorB.getSpeed());
        Delay.msDelay(2000);
        motorA.stop();
        motorB.stop();
        
        total_distance += motorA_position - motorA.getPosition();
        motorA_position = motorA.getPosition();
        motorB_position = motorB.getPosition();
        
        System.out.println("Stop");
        System.out.println("Backward");
        motorA.backward();
        motorB.backward();
        System.out.println("Large Motor A is moving: " + motorA.isMoving() + " at speed {}" + motorA.getSpeed());
        System.out.println("Large Motor B is moving: " +  motorB.isMoving() + " at speed {}" + motorB.getSpeed());
        Delay.msDelay(2000);
        System.out.println("Stop");
        motorA.stop();
        motorB.stop();
        
        total_distance += motorA_position - motorA.getPosition();
        motorA_position = motorA.getPosition();
        motorB_position = motorB.getPosition();
        
        System.out.println("Forward");
        motorA.forward();
        motorB.forward();
        Delay.msDelay(2000);
        motorA.stop();
        motorB.stop();
        
        total_distance += motorA_position - motorA.getPosition();
        motorA_position = motorA.getPosition();
        motorB_position = motorB.getPosition();
        
        motorA.close();
        motorB.close();
        
        System.out.println("Stop");
        
        System.out.println("Generating battery/robot readings");
        float battery_current = robot_battery.getBatteryCurrent();
        float motor_current = robot_battery.getMotorCurrent();
        float voltage_current = robot_battery.getVoltage();
        float millivoltage_current = robot_battery.getVoltageMilliVolt();
        
        System.out.println("Battery is: " + battery_current);
        System.out.println("Motor current is: " + motor_current);
        System.out.println("Voltage is: " + voltage_current);
        System.out.println("Millivolts is: " + millivoltage_current);
        System.out.println("Total Distance is: " + total_distance);
        
        Delay.msDelay(5000); // wait 5 seconds to get readings
		System.out.println("Press any button to exit.");
		Button.waitForAnyPress();
        

	}

}
