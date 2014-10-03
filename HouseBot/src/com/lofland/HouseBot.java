package com.lofland;

/* Version 2.0
 * Written more or less from scratch
 * and multi threaded.
 */

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.lang.Thread;

import com.lofland.shared.HMC5883L;
import com.lofland.shared.HMC5883L.Samples.AngleUnits;
import com.lofland.shared.RobotFunctions;

import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXT;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;
import lejos.util.PilotProps;
import lejos.util.Stopwatch;

public class HouseBot {

	/*
	 * Robot constants:
	 */
	static final int TIMEOUTINSECONDS = 10 * 60; // How long to wait idle with no connection before poweroff
	static final int MINSTARTDISTANCE = 10;
	static final int HALTDISTANCE = 25;
	static final int MAXCOURSEDIFFERENCE = 5; // How far off course before we start correcting?
	static final int LOOPDELAY = 35; // How much time between checking distance and adjusting?
	// The SensorThead takes at least 30ms to capture all 4 times right now
	static final int MAXSPEED = 70; // This is a percentage of 100 total
	// Speeds above 70% are not accurate

	final static File R2D2WHISTLE = new File("R2D2d4NXT.wav");

	/*
	 * static variables can be accessed anywhere
	 */
	static Command commandFromAndroid;
	static Command commandFromRobot = Command.EMPTY; // For when the Robot has his own ideas! :)
	static boolean okToGo = false;
	static int[] valuesFromAndroid = new int[10];
	static volatile boolean tabletConnected = false;
	static volatile Stopwatch readThreadTimeout = new Stopwatch();

	/*
	 * lastResult and robotIsDoing should almost always be set in main motor thread, Even if there is nothing to do there, it always gets to set the command to empty, so it can
	 * also set the lastResult or robotIsDoing.
	 */
	static String lastResult = "EMPTY";
	static String robotIsDoing = "NOTHING";
	// Initial camera angle. I think this should face forward
	static int defaultCameraAngle = 0; // Let Android set this to a new default on connect or change
	static int forwardCameraPosition; // For use later

	/*
	 * These variables will be set in the SensorThread and be used by anyone else who needs sensor data.
	 */
	static volatile int[] sensorArray = new int[4];
	static volatile int distanceCenter = 255;
	static volatile int distanceLeft = 255;
	static volatile int distanceRight = 255;
	static volatile int currentHeading = 0;
	static volatile String headingString = "MTY";
	static volatile boolean wallLeft = false;
	static volatile boolean wallRight = false;

	/*
	 * This thread will read data from Android, allowing the robot to get new commands while doing stuff.
	 * 
	 * This thread should NEVER do anything. This thread should monitor variables closely to keep in step with the main thread.
	 */
	static class ReadDataThread extends Thread {
		BTConnection connection;
		DataInputStream dataIn;
		DataOutputStream dataOut;

		public ReadDataThread() {
			super();
		}

		public void run() {
			LCD.drawString("Running . . .", 0, 0);
			while (true) {
				LCD.clear(1);
				// If the connection is lost, we want to wait again.
				// LCD.clear(6);
				// LCD.drawString("Debug 3", 0, 6);
				connect();
				while (tabletConnected) {
					// Careful, don't run connect() twice. ;)
					// LCD.clear(6);
					// LCD.drawString("Debug 2", 0, 6);
					while (commandFromAndroid == Command.EMPTY && tabletConnected) {
						if (commandFromRobot != Command.EMPTY) {
							/*
							 * If the Robot has something it wants to do, then do it instead of reading input.
							 */
							commandFromAndroid = commandFromRobot; // This makes me think the variable "FromAndroid" isn't named quite right.
							okToGo = true; // Let it go!
							commandFromRobot = Command.EMPTY; // Clear it.
						} else {
							okToGo = false;
							readThreadTimeout.reset();
							readData();
							// LCD.clear(6);
							// LCD.drawString("Debug 1", 0, 6);
							Delay.msDelay(10); // Take it easy?
							/*
							 * 250 was what I had before, but if Android is sending out ever 100, that is a problem. 50 Seems to work fine. with 200 on Android Trying 10
							 */
						}
					}
				}
				// At this point we presume the connection was lost somehow
				Sound.beepSequence();
				connection.close();
			}
		}

		public void connect() {
			/*
			 * Relax all of the motors so they don't whine or eat the battery when there is no connection
			 */
			Motor.B.flt(true);
			Motor.C.flt(true);
			// Reset default to 0, so that new value from Android will cause a change on connection
			defaultCameraAngle = 0;
			// Set down the "head" and relax it
			Motor.A.rotateTo(0);
			Motor.A.flt(true);
			// Make a "ready to connect" sound so we know it is ready
			int oldVolume = Sound.getVolume(); // Save default volume
			Sound.setVolume(75); // WAV files are very quiet
			Sound.playSample(R2D2WHISTLE, 75); // 100% of set volume
			Sound.setVolume(oldVolume); // Return to default volume.
			LCD.clear(1);
			LCD.drawString("Waiting . . .", 0, 1);
			// This method is very patient.
			connection = Bluetooth.waitForConnection();
			Sound.beepSequenceUp(); // Tell us the connection is working
			LCD.clear(1);
			LCD.drawString("Connected!", 0, 1);
			// LCD.clear(2); // Not really useful at this point, things happen
			// too fast.
			// LCD.drawString("Read: ", 0, 2);
			dataIn = connection.openDataInputStream();
			dataOut = connection.openDataOutputStream();
			readThreadTimeout.reset();
			tabletConnected = true;
		}

		private void readData() {
			int code;
			try {
				// LCD.clear(7, 2, 3); // Not really useful at this point,
				// things happen too fast.
				// LCD.drawInt(command.ordinal(), 7, 2);
				// LCD.clear(5);
				// LCD.drawString("Reading . . .", 0, 5);
				code = -1;
				try {
					code = dataIn.readInt();
				} catch (IOException e) {
					LCD.clear(1);
					LCD.clear(5);
					LCD.drawString("Read exception (1).", 0, 5);
					tabletConnected = false;
					LCD.clear(6);
					return;
				}
				if (code == -1) {
					LCD.clear(1);
					LCD.clear(5);
					LCD.drawString("code == -1", 0, 5);
					tabletConnected = false;
					// LCD.clear(6);
					return;
				}
				// LCD.clear(5);
				if (code < Command.values().length) {
					commandFromAndroid = Command.values()[code];
					// LCD.clear(7, 2, 3); // Not really useful at this point,
					// things happen too fast.
					// LCD.drawInt(command.ordinal(), 7, 2);
					/*
					 * Every command will include the same number of reads now. Some will not always be used, some will vary in their use, others will always be the same
					 */
					// Currently 4 items come in:
					// Variable (command specific, but always something):
					valuesFromAndroid[0] = dataIn.readInt();
					// travelSpeed:
					valuesFromAndroid[1] = dataIn.readInt(); // Rely on the array position for what is what
					// rotateSpeed:
					valuesFromAndroid[2] = dataIn.readInt();
					// viewAngle:
					valuesFromAndroid[3] = dataIn.readInt();
					// Done reading, now what?

					/* See if the viewAngle is new, and if so adjust it */
					if (defaultCameraAngle != valuesFromAndroid[3]) {
						Sound.beep();
						defaultCameraAngle = valuesFromAndroid[3];
						Motor.A.rotateTo((int) (forwardCameraPosition * (defaultCameraAngle / 100.0)));
					}
					/*
					 * We will do something different here based on what the command is. All data has been read, so we just use it!
					 * 
					 * The main thread is waiting for command to not be Command.EMPTY before doing anything, but it is ALSO waiting for "okToGo" to be true, That lets us get some
					 * other work done, like reading more parameters before we send the bot on its way.
					 */
					switch (commandFromAndroid) {
					/*
					 * Each case MUST include a report() as Android is awaiting a response. Otherwise everything goes wonky!
					 * 
					 * Send the report ASAP. You can delay it if you need to respond to the request with data, but if the command is asynchronous, then just send the report right
					 * away.
					 * 
					 * If you need to wait, wait on the main thread to set command to EMPTY.
					 * 
					 * Eventually EVERY command should return immediately. STATUS should tell Android if the bot is too busy to take a new command and/or if a command is received
					 * and the bot is busy it should know that and report that status to Android.
					 * 
					 * Keep the communication going and if it stops, something is wrong, try to recover (stop, talk, reset, ?)
					 * 
					 * Some of these are not used anymore.
					 */
					case EMPTY:
						/*
						 * This probably means something happened somewhere else, to break this before we got it, so we will just carry on
						 */
						break;
					case STATUS: // Dry run, because every loop ends with a standard status now.
						// Using fall through, since many all do the same thing now.
					case PROCEED:
					case STOP:
					case FORWARD:
					case LEFT:
					case RIGHT:
					case BACKWARD:
					case PING:
					case TEST:
						okToGo = true;
						break;
					case VIEWANGLE:
						/*
						 * This is an empty run since the viewangle is received on every communication now and set immediately if it is different from before. This just lets us
						 * tell the robot "do it now". Perhaps we could change the Android side to send a "STATUS" or maybe this method is fine.
						 */
						okToGo = true;
						break;
					case ROTATETOC: // Blocking
						okToGo = true;
						while (commandFromAndroid != Command.EMPTY)
							Delay.msDelay(500);
						break;
					case ROTATETOA: // Blocking
						okToGo = true;
						while (commandFromAndroid != Command.EMPTY)
							Delay.msDelay(500);
						break;
					case CALIBRATE: // Blocking
						okToGo = true;
						while (commandFromAndroid != Command.EMPTY)
							Delay.msDelay(500);
						break;
					case RESET:
						Motor.A.rotateTo(0);
						System.exit(0);
					default:
						okToGo = true;
					}
				} else
					lastResult = "ENUM out of bounds!";
			} catch (IOException e) {
				LCD.clear(1);
				LCD.clear(5);
				LCD.drawString("Read Exception!", 0, 5);
				tabletConnected = false;
				/*
				 * Too many LCD writes like this seem to use up memory, It may have to do with how it is used, as there are some notes about it in the forum. Once I commented out
				 * all of these "debug" items memory seems to be fine.
				 */
				// LCD.clear(6);
				// LCD.drawString("Debug 4", 0, 6);
				return;
			}
			report();
			// LCD.clear(6);
			// LCD.drawString("Debug 5", 0, 6);
		}

		public void report() {
			/*
			 * Make this a standard message so Android always get the same thing. and report based on what exists, not what we were told, so this does the same thing at the same
			 * time every time. Return: robotIsDoing - based on what is going on in the main thread with movement, we can use this to know if we can send new commands yet?
			 * returnInformation - whatever info the last command wanted to send when it was done Another variable - is there anything else we need to say? distanceCenter
			 * distanceLeft distanceRight Heading Tilt Anything else?
			 * 
			 * What is the best delimiter? Comma I think ","
			 */
			//TODO Turn this into JSON and use a JSON parser!
			String messageToAndroid = "{"
			        + "\"status\":\""
			        + "OK"
			        + "\",\"distanceCenter\":"
			        + distanceCenter
			        + ",\"distanceLeft\":"
			        + distanceLeft
			        + ",\"distanceRight\":"
			        + distanceRight
			        + ",\"Heading\":\""
			        + headingString
			        + "\",\"robotIsDoing\":\""
			        + robotIsDoing
			        + "\",\"lastResult\":\""
			        + lastResult
			        + "\",\"tilt\":\""
			        + "TILT"
			        + "\",\"nothing\":\""
			        + "Nothing to See here"
			        + "\"}";
			/*
			 * I think there is some sort of other thing we need to report along the lines of, "you asked for this, and this is what I heard and am doing." Kind of a "last
			 * action" not the same as "isdoing" or "returnInformation"?
			 */

			try {
				dataOut.writeUTF(messageToAndroid);
				dataOut.flush();
				// LCD.clear(4);
				// LCD.drawString(messageToAndroid, 0, 4);
			} catch (IOException e) {
			}
		}
	}

	static class SensorThread extends Thread {
		
		private DifferentialPilot pilot;
		private float newTravelDistance = 0;
		private float previousTravelDistance = 0;
		private int leftWallCounter = 0;
		private int rightWallCounter = 0;
		private int waitCount = 0; // How many seconds robot has been idle with no connection.

		public SensorThread(DifferentialPilot pilot) {
			super();
			/* I think that this will work, because objects are
			 * passed by reference, but for heaven's sake
			 * do NOT write/modify pilot, just read data!
			 * 
			 * If there is a way to "lock" it, that would be nice,
			 * for now, just be smart.
			 */
			this.pilot = pilot;
		}

		// Reusable cod to get heading
		private void setHeading(HMC5883L compass) {
			float tempHeading;
			tempHeading = compass.getHeading(AngleUnits.Radians);
			currentHeading = RobotFunctions.convertHeading(tempHeading);
			if (headingString != "CAL")
				headingString = currentHeading + "";
		}

		// Reusable cod to get distance from any Ultra Sonic Sensor
		private int setDistance(UltrasonicSensor sonar, int previousReading) {
			int distance;
			sonar.setMode(UltrasonicSensor.MODE_PING);
			// Sensor returns to off after reading
			distance = sonar.getDistance();
			// getDistance takes whatever time it needs
			if (previousReading < 255) { // If the previous reading was not 255
				if (distance > 254) { // and this one is, take another reading to flush out spurious errors
					sonar.setMode(UltrasonicSensor.MODE_PING); // Sensor returns to off after reading
					distance = sonar.getDistance(); // getDistance takes whatever time it needs
				} // If we get 255 twice in a row, it is probably real
			}
			return distance;

		}

		public void run() {
			HMC5883L compass = new HMC5883L(SensorPort.S1);
			
			/* Check out this thread for some practical notes and ideas
			 * about the compass:
			 * http://www.lejos.org/forum/viewtopic.php?p=6818
			 */
			// Set compass parameters based on location:
			/*
			 * http://magnetic-declination.com/ WICHITA KANSAS Latitude: 37° 41' 32.1" N Longitude: 97° 20' 15.1" W Magnetic declination: 3° 56' EAST Declination is POSITIVE
			 * Inclination: 65° 44' Magnetic field strength: 52047.0 nT
			 */
			compass.setDeclination(3, 56); // Used in getHeading to convert magnetic North to "true" North
			compass.setInclination(65, 44); // I do NOT think this is actually used
			compass.setFieldStrength(52047 / 100); // (100 nT =1 milliGuass) Used by isDisturbed
			// compass.setUndesturbedMargin(0.1f); // 0.1f = 10% above or below local field will be considered "disturbed"

			if (compass.offset[0] == 0)
				headingString = "CAL";

			UltrasonicSensor sonarCenter = new UltrasonicSensor(SensorPort.S4);
			sonarCenter.setMode(UltrasonicSensor.MODE_OFF);
			UltrasonicSensor sonarLeft = new UltrasonicSensor(SensorPort.S3);
			sonarLeft.setMode(UltrasonicSensor.MODE_OFF);
			UltrasonicSensor sonarRight = new UltrasonicSensor(SensorPort.S2);
			sonarRight.setMode(UltrasonicSensor.MODE_OFF);

			while (true) {
				if (tabletConnected) {
					waitCount = 0;
					if (readThreadTimeout.elapsed() > 10000) {
						tabletConnected = false;
						/*
						 * This will gracefully disconnect the robot if it does not see any data from Android often enough. Giving us an easy exit if something goes wrong on the
						 * Android side. We could even make this reset?
						 * 
						 * This doesn't cover just being stuck in the read thread, as it won't ever see the "tabletConnected" variable.
						 * 
						 * Other ideas: watchdog for many items to prevent running into walls, or being stuck, or power off if left idle.
						 */
					} else {
						// UC Sensors are slow, but I think we need to update this more often
						setHeading(compass);
						distanceCenter = setDistance(sonarCenter, distanceCenter);
						Delay.msDelay(5); // 20ms minimum delay between each sensor, but we have 3, so it should be fine.

						// UC Sensors are slow, but I think we need to update this more often
						setHeading(compass);
						distanceLeft = setDistance(sonarLeft, distanceLeft);
						Delay.msDelay(5); // 20ms minimum delay between each sensor, but we have 3, so it should be fine.

						// UC Sensors are slow, but I think we need to update this more often
						setHeading(compass);
						distanceRight = setDistance(sonarRight, distanceRight);
						Delay.msDelay(5); // 20ms minimum delay between each sensor, but we have 3, so it should be fine.
						
						// Wall Check:
						// TODO:
						/* Now I need a way to send debug output
						 * to the Android and display it.
						 * Must be time for another status field in the comm
						 * thread and another text display field in the app
						 */
						if (distanceLeft < 255) {
							/* It appears that DifferentialPilot.getTravelDistance() went away,
							 * and now we have to use .getMoveIncrement()
							 * http://www.lejos.org/forum/viewtopic.php?f=7&t=3195
							 * I'm not totally sure what "since last started moving" means yet
							 */
							newTravelDistance = pilot.getMovementIncrement();
							if ( (newTravelDistance - previousTravelDistance) > 1.0) {
								if (leftWallCounter > 5) {
									wallLeft = true;
								} else {
									leftWallCounter++;
								}
								previousTravelDistance = newTravelDistance;
							}
						} else {
							leftWallCounter = 0;
							wallLeft = false;
						}
					}

					/*
					 * String comparison in Java: http://stackoverflow.com/questions/513832/how-do-i-compare-strings-in-java
					 * 
					 * == tests for reference equality. .equals() tests for value equality. Consequently, if you actually want to test whether two strings have the same value you
					 * should use .equals() (except in a few situations where you can guarantee that two strings with the same value will be represented by the same object eg:
					 * String interning).
					 * 
					 * == is for testing whether two strings are the same object.
					 * 
					 * // These two have the same value new String("test").equals("test") // --> true
					 * 
					 * // ... but they are not the same object new String("test") == "test" // --> false
					 * 
					 * // ... neither are these new String("test") == new String("test") // --> false
					 * 
					 * // ... but these are because literals are interned by // the compiler and thus refer to the same object "test" == "test" // --> true
					 * 
					 * // concatenation of string literals happens at compile time, // also resulting in the same object "test" == "te" + "st" // --> true
					 * 
					 * // but .substring() is invoked at runtime, generating distinct objects "test" == "!test".substring(1) // --> false
					 * 
					 * // interned strings can also be recalled by calling .intern() "test" == "!test".substring(1).intern() // --> true It is important to note that == is much
					 * cheaper than equals() (a single reference comparison instead of a loop), thus, in situations where it is applicable (i.e. you can guarantee that you are only
					 * dealing with interned strings) it can present an important performance improvement. However, these situations are rare.
					 */

					if (robotIsDoing.equals("FORWARD") && distanceCenter <= HALTDISTANCE) {
						/*
						 * This will halt a forward operation if the front sensor sees something, using the commandFromRobot variable.
						 */
						commandFromRobot = Command.STOP;
					} else if (robotIsDoing.equals("FORWARD") && distanceCenter < valuesFromAndroid[1]) {
						commandFromRobot = Command.FORWARD;
					}

				} else {
					if (waitCount > TIMEOUTINSECONDS)
						NXT.shutDown(); // Preserve batteries by not staying on all day
					/*
					 * If the system is not connected, then just rest a while. :)
					 */
					Delay.msDelay(1000);
					waitCount++;
				}
			}
		}

	}

	/*
	 * The main thread will DO all of the "stuff" because the robot can never do two things at once.
	 * 
	 * This thread should watch variables closely so that it doesn't step on the read thread.
	 */
	public static void main(final String args[]) {
		LCD.clear();
		commandFromAndroid = Command.EMPTY;
		float moveSpeed; // All of our routines can use this, but it cannot be
							// molested by other threads
		int currentHeading = 0;
		int previousHeading = 0;
		int headingDifference = 0;
		int absHeadingDifference = 0;

		PilotProps pp = new PilotProps();
		try {
			pp.loadPersistentValues();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Printed on the side of the wheel:
		float wheelDiameter = Float
				.parseFloat(pp.getProperty(PilotProps.KEY_WHEELDIAMETER, "94.8"));
		// Measured from midpoint to midpoint:
		float trackWidth = Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, "185.0"));
		// if it is spinning too far, lower it, not far enough, increase it.

		RegulatedMotor leftMotor = PilotProps.getMotor(pp
				.getProperty(PilotProps.KEY_LEFTMOTOR, "B"));
		RegulatedMotor rightMotor = PilotProps.getMotor(pp.getProperty(PilotProps.KEY_RIGHTMOTOR,
				"C"));
		boolean reverse = Boolean.parseBoolean(pp.getProperty(PilotProps.KEY_REVERSE, "false"));
		DifferentialPilot pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor,
				rightMotor, reverse);

		// Set the camera position on every robot startup.
		forwardCameraPosition = RobotFunctions.cameraPositionSet();

		new SensorThread(pilot).start();
		new ReadDataThread().start();
		while (true) {
			Delay.msDelay(10); // Making this too long makes robot unresponsive
			// LCD.clear(3);
			// LCD.drawString("Exec: ", 0, 3);
			// LCD.drawInt(command.ordinal(), 7, 3);
			/*
			 * We need the command to be non-Empty, AND we need the "all clear" from the reader that it is done reading done via the "okToGo" variable Some commands take more reads
			 */
			if (commandFromAndroid != Command.EMPTY && okToGo) {
				switch (commandFromAndroid) {
				/*
				 * The reader will not resume until command = Command.EMPTY, so we have the opportunity to decide when we allow new commands to be received.
				 * 
				 * Normally set it to EMPTY as LATE as possible, because new commands will start to come in at that point and we don't want to make a mess. So keep the reader
				 * closed until we are done unless we have a very good reason to let it start again sooner, such as when we have a loop to start.
				 * 
				 * Also, be SURE you have set any report information into the variable before setting command to EMPTY!
				 * 
				 * Some commands may be async, as in they are sent, we run them, and Android is free to send other commands. FORWARD and PROCEED are like that, at least for now.
				 * 
				 * Others will be linear, meaning Android expects to have to wait for a report until the operation is done. VIEWANGLE is like that, as it should perform a clear
				 * operation and then be done.
				 * 
				 * Some of these are not used anymore.
				 */
				case STATUS:
					// The comm thread does the work, because this shouldn't
					// stop other tasks happening here.
					commandFromAndroid = Command.EMPTY;
					break;
				case PROCEED:
					robotIsDoing = "PROCEED";

					// Reset defaults if needed!

					// We need a special way to keep this alive
					boolean happyToProceed = true;
					commandFromAndroid = Command.EMPTY; // Let the comm thread proceed now.
					/*
					 * Remember, now you have a "mini thread" here, where you have to do all of the work that was being done above. Other calls may come in and you have to handle
					 * them correctly.
					 */
					// Record heading
					previousHeading = currentHeading;
					while (happyToProceed) {
						Delay.msDelay(LOOPDELAY);

						headingDifference = RobotFunctions.GetHeadingError(previousHeading,
								currentHeading);
						absHeadingDifference = Math.abs(headingDifference);

						// Use one instance to compare, otherwise each one is new!
						int proceedDistance = distanceCenter;

						if (commandFromAndroid == Command.STOP && okToGo) {
							/*
							 * We really have to have both, otherwise we get a race condition. The entire point of this system is so that the two threads coordinate! We are not
							 * allowed to modify command until okToGo!
							 */
							lastResult = "PROCEED received STOP command";
							happyToProceed = false;
							commandFromAndroid = Command.EMPTY;
						} else if (absHeadingDifference > 90) {
							robotIsDoing = "OFFCOURSE " + headingDifference;
							RobotFunctions.robotStop(pilot);
							rotateRobotToCompass(pilot, previousHeading);
						} else if (proceedDistance < HALTDISTANCE) {
							happyToProceed = false;
							lastResult = "PROCEED OBSTRUCTED";
						} else {
							if (proceedDistance > MAXSPEED)
								moveSpeed = MAXSPEED;
							else
								moveSpeed = proceedDistance;
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.setRotateSpeed(moveSpeed);
							if (absHeadingDifference > MAXCOURSEDIFFERENCE)
								pilot.steer(headingDifference);
							else
								pilot.forward();
						}

						if (okToGo && commandFromAndroid != Command.STOP) {
							/*
							 * This will flush out any other commands we get. That we cannot handle because we are busy. Again, prevent a race condition with the != Command.STOP,
							 * in case it was set while we were busy, this way we don't miss it. If it is STOP, we will pick it up on the next cycle.
							 */
							lastResult = "BUSY in PROCEED";
							commandFromAndroid = Command.EMPTY;
						}
					}
					RobotFunctions.robotStop(pilot);
					robotIsDoing = "STOPPED";
					break;
				case STOP:
					robotIsDoing = "STOPPED";
					RobotFunctions.robotStop(pilot);
					commandFromAndroid = Command.EMPTY;
					lastResult = "Robot Stopped";
					break;
				case ROTATETOA: // An absolute rotation
					robotIsDoing = "ROTATETOA" + valuesFromAndroid[0];
					moveSpeed = 25; // Going too fast makes this unreliable
					moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
					pilot.setTravelSpeed(moveSpeed);
					pilot.setRotateSpeed(moveSpeed);
					pilot.rotate(valuesFromAndroid[0]); // variable field
					lastResult = "ROTATED: " + headingDifference;
					robotIsDoing = "STOPPED";
					commandFromAndroid = Command.EMPTY;
					break;
				case ROTATETOC: // Rotate to compass heading
					robotIsDoing = "ROTATETOC" + valuesFromAndroid[0]; // variable
																		// field
					rotateRobotToCompass(pilot, valuesFromAndroid[0]); // variable
																		// field
					lastResult = "Rotated to " + valuesFromAndroid[0];
					robotIsDoing = "STOPPED";
					commandFromAndroid = Command.EMPTY;
					break;
				case FORWARD:
					robotIsDoing = "FORWARD";
					if (distanceCenter > MINSTARTDISTANCE) {
						if (distanceCenter > valuesFromAndroid[1])
							moveSpeed = valuesFromAndroid[1]; // travelSpeed
						else
							moveSpeed = distanceCenter;
						moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
						// I should experiment to see if I'm really getting the max travel speed at 100!
						pilot.setTravelSpeed(moveSpeed);
						pilot.forward();
						lastResult = "FORWARD";
					} else {
						RobotFunctions.robotStop(pilot);
						robotIsDoing = "STOPPED";
						lastResult = "FORWARD Obstructed!";
					}
					commandFromAndroid = Command.EMPTY;
					break;
				case LEFT:
					robotIsDoing = "LEFT";
					moveSpeed = valuesFromAndroid[2]; // rotateSpeed
					moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
					pilot.setTravelSpeed(moveSpeed);
					pilot.arcForward(0);
					lastResult = "LEFT";
					commandFromAndroid = Command.EMPTY;
					break;
				case RIGHT:
					robotIsDoing = "RIGHT";
					moveSpeed = valuesFromAndroid[2]; // rotateSpeed
					moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
					pilot.setTravelSpeed(moveSpeed);
					pilot.arcBackward(0);
					lastResult = "RIGHT";
					commandFromAndroid = Command.EMPTY;
					break;
				case BACKWARD:
					robotIsDoing = "BACKWARD";
					moveSpeed = valuesFromAndroid[1]; // travelSpeed
					moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
					pilot.setTravelSpeed(moveSpeed);
					pilot.backward();
					lastResult = "BACKWARD";
					commandFromAndroid = Command.EMPTY;
					break;
				case VIEWANGLE:
					lastResult = "View Angle Set";
					commandFromAndroid = Command.EMPTY;
					break;
				case PING:
					// Do something useful with "PING" :)
					lastResult = Runtime.getRuntime().freeMemory() + "";
					commandFromAndroid = Command.EMPTY;
					break;
				case TEST:
					// This is to test stuff
					// Currently testing a wander using all 3 sensors
					robotIsDoing = "TEST";

					// Adjust defults if needed

					// We need a special way to keep this alive
					boolean happyToTest = true;
					commandFromAndroid = Command.EMPTY; // Let the comm thread proceed now.
					/*
					 * Remember, now you have a "mini thread" here, where you have to do all of the work that was being done above. Other calls may come in and you have to handle
					 * them correctly.
					 */

					// Record heading
					previousHeading = currentHeading;
					while (happyToTest) {
						Delay.msDelay(LOOPDELAY);

						// Don't run into walls
						if (distanceLeft < HALTDISTANCE) {
							RobotFunctions.robotStop(pilot);
							moveSpeed = 15; // Going too fast makes this
											// unreliable
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.setRotateSpeed(moveSpeed);
							while (distanceLeft < HALTDISTANCE)
								pilot.rotateRight();
							previousHeading = currentHeading; // New heading
						} else if (distanceRight < HALTDISTANCE) {
							RobotFunctions.robotStop(pilot);
							moveSpeed = 15; // Going too fast makes this
											// unreliable
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.setRotateSpeed(moveSpeed);
							while (distanceRight < HALTDISTANCE)
								pilot.rotateLeft();
							previousHeading = currentHeading; // New heading
						}

						headingDifference = RobotFunctions.GetHeadingError(previousHeading,
								currentHeading);
						absHeadingDifference = Math.abs(headingDifference);

						// Use one instance to compare, otherwise each one is
						// new!
						int testDistanceCenter = distanceCenter;

						if (commandFromAndroid == Command.STOP && okToGo) {
							/*
							 * We really have to have both, otherwise we get a race condition. The entire point of this system is so that the two threads coordinate! We are not
							 * allowed to modify command until okToGo!
							 */
							happyToTest = false;
							commandFromAndroid = Command.EMPTY;
						} else if (absHeadingDifference > 90) {
							robotIsDoing = "OFFCOURSE " + headingDifference;
							RobotFunctions.robotStop(pilot);
							rotateRobotToCompass(pilot, previousHeading);
						} else if (testDistanceCenter < HALTDISTANCE) {
							RobotFunctions.robotStop(pilot);
							moveSpeed = 15; // Going too fast makes this
											// unreliable
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.setRotateSpeed(moveSpeed);
							if (distanceRight >= distanceLeft) {
								while (distanceCenter < 255)
									pilot.rotateRight();
							} else {
								while (distanceCenter < 255)
									pilot.rotateLeft();
							}
							previousHeading = currentHeading; // New heading
						} else {
							if (testDistanceCenter > MAXSPEED)
								moveSpeed = MAXSPEED;
							else
								moveSpeed = testDistanceCenter;
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.setRotateSpeed(moveSpeed);
							if (absHeadingDifference > MAXCOURSEDIFFERENCE)
								pilot.steer(headingDifference);
							else
								pilot.forward();
						}

						if (okToGo && commandFromAndroid != Command.STOP) {
							/*
							 * This will flush out any other commands we get. That we cannot handle because we are busy. Again, prevent a race condition with the != Command.STOP,
							 * in case it was set while we were busy, this way we don't miss it. If it is STOP, we will pick it up on the next cycle.
							 */
							commandFromAndroid = Command.EMPTY;
						}
					}
					RobotFunctions.robotStop(pilot);
					robotIsDoing = "STOPPED";
					break;
				case STAYCLOSE:
					robotIsDoing = "STAYCLOSE";

					// Reset defaults if needed!

					// We need a special way to keep this alive
					boolean happyToFollow = true;
					commandFromAndroid = Command.EMPTY; // Let the comm thread proceed now.

					while (happyToFollow) {
						Delay.msDelay(LOOPDELAY);
						// Use one instance to compare, otherwise each one is
						// new!
						int distance = distanceCenter;

						if (commandFromAndroid == Command.STOP && okToGo) {
							/*
							 * We really have to have both, otherwise we get a race condition. The entire point of this system is so that the two threads coordinate! We are not
							 * allowed to modify command until okToGo!
							 */
							happyToFollow = false;
							lastResult = "STAYCLOSE Stopped by user";
							commandFromAndroid = Command.EMPTY;
						} else if (distance > 254) {
							// 255 = no echo return
							// Error or Out of range
							// Here we could look left/right for something!
							RobotFunctions.robotStop(pilot);
							robotIsDoing = "TOOFAR";
							// Distances are in centimeters
						} else if (distance > 25) {
							if (distance > MAXSPEED)
								moveSpeed = MAXSPEED;
							else
								moveSpeed = distance;
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.setRotateSpeed(moveSpeed);
							if (absHeadingDifference > MAXCOURSEDIFFERENCE)
								pilot.steer(headingDifference);
							else
								pilot.forward();
							robotIsDoing = "STAYCLOSE";
						} else if (distance < 20) {
							moveSpeed = 25;
							moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
							pilot.setTravelSpeed(moveSpeed);
							pilot.backward();
							robotIsDoing = "BACKOFF";
						} else {
							// Happy spot :)
							RobotFunctions.robotStop(pilot);
							robotIsDoing = "HAPPYSPOT";
						}

						if (okToGo && commandFromAndroid != Command.STOP) {
							/*
							 * This will flush out any other commands we get. That we cannot handle because we are busy. Again, prevent a race condition with the != Command.STOP,
							 * in case it was set while we were busy, this way we don't miss it. If it is STOP, we will pick it up on the next cycle.
							 */
							lastResult = "BUSY in STAYCLOSE";
							commandFromAndroid = Command.EMPTY;
						}
					}
					RobotFunctions.robotStop(pilot);
					robotIsDoing = "STOPPED";
					break;
				case FINDCLEARLEFT:
					robotIsDoing = "FINDCLEARLEFT";
					moveSpeed = valuesFromAndroid[1]; // travelSpeed
					moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
					pilot.setTravelSpeed(moveSpeed);
					pilot.arcForward(0);
					lastResult = "FINDCLEARLEFT done";
					commandFromAndroid = Command.EMPTY;
				case FINDCLEARRIGHT:
					break;
				default:
					RobotFunctions.robotStop(pilot);
					robotIsDoing = "STOPPED";
					lastResult = "Fell into default!";
					commandFromAndroid = Command.EMPTY;
				}
				// LCD.clear(5);
				// LCD.clear(6);
				// LCD.drawString("Debug 4", 0, 6);
			}
		}
	}

	/*
	 * These reusable functions should go to the RobotFunctions.java class so that they can be used in LeJOStest apps and such.
	 */

	private static void rotateRobotToCompass(DifferentialPilot pilot, int newHeading) {
		float moveSpeed = 25; // Going too fast makes this unreliable
		moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
		pilot.setTravelSpeed(moveSpeed);
		pilot.setRotateSpeed(moveSpeed);
		/*
		 * Slip and over run can cause it to not make it all of the way or overshoot. By repeating the same scenario you can get closer. My experiments show that 6 rounds works
		 * even on very poor traction situations.
		 */
		int retryLoops = 0;
		while (retryLoops < 6) { // 6 times
			int headingDifference = RobotFunctions.GetHeadingError(currentHeading, newHeading);
			pilot.rotate(-headingDifference); // Rotate OPPOSITE of difference
												// to eliminate it
			if (retryLoops == 0) // I want the first instance to report, the
									// highest.
				lastResult = "ROTATED " + headingDifference;
			Delay.msDelay(10); // Compass can only return headings so quickly
			retryLoops++;
		}
	}

	/*
	 * Memory: Each enum is an object, so they each take like 4 bytes? http://www.lejos.org/forum/viewtopic.php?f=7&t=3269 So in theory I might need to go to just using straight
	 * integers instead, but so far my memory usage seems fine.
	 */
	enum Command // copied from GridNavControl project
	/*
	 * These have to match what is in the Android side, in order and location, though the actual text doesn't have to be the same, since they are converted to numbers on both sides
	 * of the connection and sent over as integers. In theory I could just use the integers in a switch statement instead, but this is more pretty.
	 */
	{
		EMPTY, STATUS, PROCEED, FORWARD, LEFT, RIGHT, BACKWARD, STOP, BEHAVE, PING, TEST, VIEWANGLE, STAYCLOSE, RESET, FINDCLEARRIGHT, FINDCLEARLEFT, ROTATETOA, ROTATETOC, CALIBRATE;
	}
}
