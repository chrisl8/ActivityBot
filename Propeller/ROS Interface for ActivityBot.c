/*
 ActivityBot (AB) code to connect to Dr. Rainer Hessmer's
 ROS with Arduino project: https://code.google.com/p/drh-robotics-ros/
 This code is largely a sloppy "conversion" of Dr. Hessmer's robot.pde for the Arduino:
 https://code.google.com/p/drh-robotics-ros/source/browse/trunk/Arduino/Robot/Robot.pde
 Dr. Hessmer's blog explaining how his code and robot works is here:
 http://www.hessmer.org/blog/2010/11/21/sending-data-from-arduino-to-ros/

 Please also see these sites which helped me tremendously with the formulas:
 http://www.seattlerobotics.org/encoder/200610/article3/IMU%20Odometry,%20by%20David%20Anderson.htm
 http://webdelcire.com/wordpress/archives/527
 And of course the entire point of this is to interface with ROS, so read about everything ROS here:
 http://wiki.ros.org/
 All code here heavily borrowed from everywhere code can be found! :)
 See "Serial Testing for ROS ActivityBot 1-4" (which are not on GitHub) for previous versions and explanations.
 */

#include "simpletools.h"
#include "fdserial.h"
#include "ping.h"                             // Include ping header
/*
http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1277133&viewfull=1#post1277133
"The most impressive is that we can use the same code as the ActivityBot examples, replacing only the library’s name. So everyone who has a big-size Robot, similar to The Artist (like Arlo, Eddie or any other) must only change the library “abdrive.h” with the “arlodrive.h”. So we can take all the advantages form the pre-written code for the ActivityBot!'
http://www.parallax.com/news/2013-12-23/run-activitybot-c-code-arlo
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&posted=1#post1230592
*/
#include "abdrive.h"

fdserial *term;

// Robot description: We will get this from ROS so that it is easier to tweak between runs without reloading the AB EEPROM.
static double distancePerCount = 0.0; // See encoders.yaml to set or change this value
static double trackWidth = 0.0; // See encoders.yaml to set or change this value

const char delimiter[2] = ","; // Delimiter character for incoming messages from the ROS Python script

// For Odometry
int ticksLeft, ticksRight, ticksLeftOld, ticksRightOld;
static double Heading = 0.0, X = 0.0, Y = 0.0;
static int speedLeft, speedRight;

void getTicks();
void displayTicks();
void drive_getSpeedCalc(int *left, int *right);

void broadcastOdometry(void *par); // Use a cog to broadcast Odometry to ROS continuously
static int fstack[256]; // If things get weird make this number bigger!

// for Ping Sensors
// Name them based on the center of the direction
// in which they point in radians
// (radians because that is what sensor_msgs/LaserScan uses)
static int pingRange0 = 0;
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[256]; // If things get weird make this number bigger!


int main() {

	simpleterm_close(); // Close simplex serial terminal
	term = fdserial_open(31, 30, 0, 115200); // Open Full Duplex serial connection

	/* Wait for ROS to give us the robot parameters,
	 broadcasting 'i' until it does to tell ROS that we
	 are ready */
	int robotInitialized = 0; // Do not compute odometry until we have the trackWidth and distancePerCount
	while (robotInitialized == 0) {
		dprint(term, "i\t0\n"); // Request Robot distancePerCount and trackWidth NOTE: Python code cannot deal with a line with no divider characters on it.
		pause(10); // Give ROS time to respond, but not too much or we bump into other stuff that may be coming in from ROS.
		if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
			char buf[20]; // A Buffer long enough to hold the longest line ROS may send.
			int count = 0;
			while (count < 20) {
				buf[count] = readChar(term);
				if (buf[count] == '\r' || buf[count] == '\n')
					break;
				count++;
			}

			if (buf[0] == 'd') {
				char *token;
				token = strtok(buf, delimiter);
				token = strtok(NULL, delimiter);
				char *unconverted;
				trackWidth = strtod(token, &unconverted);
				token = strtok(NULL, delimiter);
				distancePerCount = strtod(token, &unconverted);
				if (trackWidth > 0.0 && distancePerCount > 0.0)
					robotInitialized = 1;
			}
		} else {
			pause(500); // Longer pauses when robot is uninitialized
		}
	}
    
    // Start the sensor cog(s)
	cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);

	// Now initialize the Motors
	// abdrive settings:
	drive_speed(0, 0);                     // Start servos/encoders cog
	//drive_setRampStep(10);              // Set ramping at 10 ticks/sec per 20 ms
	// TODO Do we need to adjust ramping? Perhaps this should be something we can modify on the ROS side and send?

	// Start the Odometry broadcast cog
	cogstart(&broadcastOdometry, NULL, fstack, sizeof fstack);

	// To hold received commands
	double CommandedVelocity = 0.0;
	double CommandedAngularVelocity = 0.0;

	// Listen for drive commands
	while (1) {

		/* TODO:
		 1. Should there should be code here to stop the motors if we go too long with no input from ROS?
		 2. There should be a way to reset the odometry to 0 remotely, for when we pick up the robot and start over, so we don't have to press the reset button on the AB every time.
		 */

		if (fdserial_rxReady(term) != 0) { // Non blocking check for data in the input buffer
			char buf[20]; // A Buffer long enough to hold the longest line ROS may send.
			int count = 0;
			while (count < 20) {
				buf[count] = readChar(term);
				if (buf[count] == '\r' || buf[count] == '\n')
					break;
				count++;
			}

			if (buf[0] == 's') {
				char *token;
				token = strtok(buf, delimiter);
				token = strtok(NULL, delimiter);
				char *unconverted;
				CommandedVelocity = strtod(token, &unconverted);
				token = strtok(NULL, delimiter);
				CommandedAngularVelocity = strtod(token, &unconverted);
				double angularVelocityOffset = 0.5 * CommandedAngularVelocity * trackWidth;
				double expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
				double expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

				expectedLeftSpeed = expectedLeftSpeed / distancePerCount;
				expectedRightSpeed = expectedRightSpeed / distancePerCount;

				drive_speed(expectedLeftSpeed, expectedRightSpeed);
			}
		}
		pause(10); // Maximum read frequency. TODO: Is this required? Is it the right length?
	}
}

/* Some of the code below came from Dr. Rainer Hessmer's robot.pde
 The rest was heavily inspired/copied from here:
 http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
 */
void broadcastOdometry(void *par) {

	int dt = CLKFREQ / 10;
	int t = CNT;

	while (1) {
		if (CNT - t > dt) {
			t += dt;
			getTicks();
			displayTicks();
		}
	}
}

void getTicks(void) {
	ticksLeftOld = ticksLeft;
	ticksRightOld = ticksRight;
	drive_getTicks(&ticksLeft, &ticksRight);
	drive_getSpeedCalc(&speedLeft, &speedRight);
}

void displayTicks(void) {
	int deltaTicksLeft = ticksLeft - ticksLeftOld;
	int deltaTicksRight = ticksRight - ticksRightOld;
	double deltaDistance = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * distancePerCount;
	double deltaX = deltaDistance * (double) cos(Heading);
	double deltaY = deltaDistance * (double) sin(Heading);
	double RadiansPerCount = distancePerCount / trackWidth;
	double deltaHeading = (double) (deltaTicksRight - deltaTicksLeft) * RadiansPerCount;

	X += deltaX;
	Y += deltaY;
	Heading += deltaHeading;
	// limit heading to -Pi <= heading < Pi
	if (Heading > PI) {
		Heading -= 2.0 * PI;
	} else {
		if (Heading <= -PI) {
			Heading += 2.0 * PI;
		}
	}

	// http://webdelcire.com/wordpress/archives/527
	double V = ((speedRight * distancePerCount) + (speedLeft * distancePerCount)) / 2;
	double Omega = ((speedRight * distancePerCount) - (speedLeft * distancePerCount)) / trackWidth;

	dprint(term, "o\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\n", X, Y, Heading, V, Omega, pingRange0); // Odometry for ROS
    // Now with sensor ranges!!
}

volatile int abd_speedL;
volatile int abd_speedR;

void drive_getSpeedCalc(int *left, int *right) {
	*left = abd_speedL;
	*right = abd_speedR;
}

void pollPingSensors(void *par) {
      while(1)                                    // Repeat indefinitely
  {
    pingRange0 = ping_cm(8);                 // Get cm distance from Ping)))
    pause(200);                               // Wait 1/5 second
    /*
    http://forums.parallax.com/showthread.php/111215-multiple-pings?highlight=ping+round+trip+time
"The echos from one PING could confuse the others if they were operating simultaneously. You really need to trigger only one at a time and allow a little time for the echos to die down before triggering another PING even if they're facing in completely different directions.
I don't know what would be ideal for a delay, but the maximum sound round-trip time window for the PING))) is 18.5ms and I'd wait several times that, maybe 50 to 100ms."
    */
  }

}