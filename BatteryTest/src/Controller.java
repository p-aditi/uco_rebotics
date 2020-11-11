import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
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
	static int[][] map = new int[30][30];

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
		SampleProvider distance_provider = uss_sensor.getMode("Distance");
		float[] sample = new float[distance_provider.sampleSize()];
		distance_provider.fetchSample(sample, 0);
		return sample[0];

	}

	public static void Rotate(int degree) {
		pilot.rotate(degree);
		r += 90;
		if (r == 360) {
			r = 0;
		}
	}

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
				if (map[y][x + 1] != 1) {
					map[y][x + 1] = 1;
					startPathfinding();
				} else {
					MoveUp();
				}

				break;
			case 180:
				if (map[y + 1][x] != 1) {
					map[y + 1][x] = 1;
					startPathfinding();
				} else {
					MoveUp();
				}
				break;
			case 270:
				if (map[y][x - 1] != 1) {
					map[y][x - 1] = 1;
					startPathfinding();
				} else {
					MoveUp();
				}
				break;
			case 0:
				if (map[y - 1][x] != 1) {
					map[y - 1][x] = 1;
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
		
		if (x+1<map[0].length && map[y][x + 1] == 3) {
			RotateTowards(y, x + 1);
			Move(5);
			x++;
			map[x][y] = 4;
		} else if (x-1> -1 && map[y][x - 1] == 3) {
			RotateTowards(y, x - 1);
			Move(5);
			x--;
			map[x][y] = 4;
		} else if (y+1<map.length && map[y + 1][x] == 3) {
			RotateTowards(y + 1, x);
			Move(5);
			y++;
			map[x][y] = 4;
		} else if (y-1> -1 &&map[y - 1][x] == 3) {
			RotateTowards(y - 1, x);
			Move(5);
			y--;
			map[x][y] = 4;
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
				if (map[i][j] == 3 || map[i][j] == 4) {
					map[i][j] = 0;
				}
			}
			MazeSolver maze = new MazeSolver(map);
			if (maze.solve(x, y, 0, 0)) {
				for (int k = 0; k < map.length; k++) {
					for (int l = 0; l < map[k].length; l++) {
						if (maze.map[k][l] == 3) {
							map[k][l] = 3;
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
				map[i][j] = 0;
			}
		}

	}
}