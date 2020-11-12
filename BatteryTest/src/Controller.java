import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class Controller {

	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
	static SensorModes uss_sensor = new EV3UltrasonicSensor(SensorPort.S4);
	static SensorModes gyro_sensor = new EV3GyroSensor(SensorPort.S3);
	static int[][] map = new int[30][30]; //TODO update w/ grid size
	
	static final int OPEN = 0;
	static final int OBSTACLE = 1;
	static final int PATH = 3;

	static int x;
	static int y;
	static int r;
	static Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 3.17).offset(-7.62);
	static Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 3.17).offset(7.62);
	static Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
	static MovePilot pilot = new MovePilot(chassis);

	public static void main(String[] args) {
		// Set up the wheel for pilot.
////////////////////////////////////////////////////////////////////////////////////////////////

		// set up the chassis type: Differential pilot
		// Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 },
		// WheeledChassis.TYPE_DIFFERENTIAL);

////////////////////////////////////////////////////////////////////////////////////////////////

		float distanceValue = 0;
		initializeMap(29, 29, 270);// x,y,r
		resetGrid();

		int index = 0;
		while (true) {
			index++; // Won't be Needed'

			distanceValue = getDistance();
			NextMove(distanceValue);
		}

	}

	public static void printMap() {
		for (int i = 0; i < map.length; i++) {
			for (int j = 0; j < map[i].length; j++) {
				System.out.print(map[i][j] + " , ");

			}
			System.out.println("");
		}
	}

	// Gets the Distance
	public static float getDistance() {

		float distance 5.0;

		SampleProvider distance_provider = uss_sensor.getMode("Distance");
		float[] sample = new float[distance_provider.sampleSize()];
		distance_provider.fetchSample(sample, 0);
		for(int i = 0; i < distance_provider.sampleSize(); i++){
			distance += sample[i];
		}
		distance = distance / distance_provider.sampleSize();
		return distance;

	}

	public static float getAngle(){
		float angle = 0;
		SampleProvider angle_provider = gyro_sensor.getMode("Angle");
		float[] sample = new float[angle_provider.sampleSize()];
		angle_provider.fetchSample(sample,0);
		for(int i = 0; i < angle_provider.sampleSize();i++){
			angle += sample[i];
		}
		angle = angle / angle_provider.sampleSize();
		return angle;
	}

	public static void Rotate(int degree) {
		
		pilot.rotate(degree);
		int currentAngle = getAngle();
		while(currentAngle > 360){
			currentAngle = currentAngle - 360; //gyro will add or subtract past 360, correct for this.
		}
		while(currentAngle < 0){
			currentAngle = currentAngle + 360;
		}

		if(currentAngle < degree)
		{// if it couldn't get an accurate turn the first time, it likely won't the second either. we can tighten it a bit though.
			pilote.rotate((degree - currentAngle) / 2); // try to move about half the distance, to get a little more accuracy
		} 
		else if(currentAngle > degree){
			pilot.rotate((currentAngle - degree) / 2)); // try to move about half the distance, to get a little more accuracy
		}
		r += 90; // Josh likely depends on this somewhere, but you could set it based off gyro, youd have to adjust for inaccuracies.
		if (r == 360) {
			r = 0;
		}
	}



   /*
	public static void Rotate(int degree) {
		
		pilot.rotate(degree);
		
		r += 90;
		if (r == 360) {
			r = 0;
		}
	}
	*/


	// moves an amount int centimeters.
	public static void Move(int amount) {
		pilot.travel(amount);
		switch (r) {
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

	public static void NextMove(float distanceVal) {

		if (distanceVal < .1) {
			// Mark object as blocked. If the object is incorrectly marked in the map,
			// restart the algorithm and mark the path.
			switch (r) {
			case 90:
				if (map[y][x + 1] != OBSTACLE) { //is there obstacle on map?
					map[y][x + 1] = OBSTACLE; // mark map
					startPathfinding(); //find new path
				} else {
					MoveUp();
				}

				break;
			case 180:
				if (map[y + 1][x] != OBSTACLE) {
					map[y + 1][x] = OBSTACLE;
					startPathfinding();
				} else {
					MoveUp();
				}
				break;
			case 270:
				if (map[y][x - 1] != OBSTACLE) {
					map[y][x - 1] = OBSTACLE;
					startPathfinding();
				} else {
					MoveUp();
				}
				break;
			case 0:
				if (map[y - 1][x] != OBSTACLE) {
					map[y - 1][x] = OBSTACLE;
					startPathfinding();
				} else {
					MoveUp();
				}
				break;

			}

		}

		else {
			MoveUp();
		}
	}

	// Moves Towards the Next "3" marked by Pathfinder.java
	public static void MoveUp() {

		if (map[y][x + 1] == PATH) { 
			RotateTowards(y, x + 1);
			Move(5);
			
			map[x][y] = OPEN;
		} else if (map[y][x - 1] == PATH) {
			RotateTowards(y, x - 1);
			Move(5);
			
			map[x][y] = OPEN;
		} else if (map[y + 1][x] == PATH) {
			RotateTowards(y + 1, x);
			Move(5);
		
			map[x][y] = OPEN;
		} else if (map[y - 1][x] == PATH) {
			RotateTowards(y - 1, x);
			Move(5);
		
			map[x][y] = OPEN;
		}

	}

	public static void startPathfinding() {
		resetGrid();
	}

	// Rotates towards a given x and y.
	public static void RotateTowards(int localX, int localY) {
		if (localX > x) {
			if (r == 0) {
				Rotate(90);
			} else if (r == 90) {
				// Do Nothing
			} else if (r == 180) {
				Rotate(-90);
			} else if (r == 270) {
				Rotate(180);
			}
		}

		if (localX < x) {
			if (r == 0) {
				Rotate(-90);
			} else if (r == 90) {
				Rotate(180);
			} else if (r == 180) {
				Rotate(90);
			} else if (r == 270) {
				// Do Nothing
			}
		}

		if (localY < y) {
			if (r == 0) {
				Rotate(180);
			} else if (r == 90) {
				Rotate(90);
			} else if (r == 180) {
				// Do Nothing
			} else if (r == 270) {
				Rotate(-90);
			}
		}

		if (localY > y) {
			if (r == 0) {
				// Do Nothing
			} else if (r == 90) {
				Rotate(-90);
			} else if (r == 180) {
				Rotate(180);
			} else if (r == 270) {
				Rotate(90);
			}
		}
	}

	// Resets all the 4s and 3s back to 0s
	public static void resetGrid() {
		for (int i = 0; i < map.length; i++) {
			for (int j = 0; j < map[i].length; j++) {
				if (map[i][j] == PATH || map[i][j] == 4) {
					map[i][j] = OPEN;
				}
			}
			MazeSolver maze = new MazeSolver(map);
			if (maze.solve(y, x, 0, 0)) {
				for (int k = 0; k < map.length; k++) {
					for (int l = 0; l < map[k].length; l++) {
						if (maze.map[k][l] == PATH) {
							map[k][l] = PATH;
						}
					}

				}
			} else {
				System.out.println("No path found");
			}
		}

	}

	public static void initializeMap(int xLocal, int yLocal, int rLocal) {

		x = xLocal;
		y = yLocal;
		r = rLocal;

		for (int i = 0; i < map.length; i++) {
			for (int j = 0; j < map[i].length; j++) {
				map[i][j] = OPEN;
			}
		}

	}
}