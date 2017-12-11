/******************************************************************************

GeoCache Hunt Project (GeoCache.ino)

This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.

Team Number: 6

Team Members: Joshua Stevens
              Joshua Suskalo

NOTES:
You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.

The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.

*******************************************************************************

Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline

Following are the results calculated from above GPS GPRMC message (current
location) to the provided GEOLAT0/GEOLON0 tree (target location).  Your
results should be nearly identical, if not exactly the same.

degMin2DecDeg() LAT_2307.1256_N = 23.118757 decimal degrees
degMin2DecDeg() LON_12016.4438_E = 120.274055 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999652 degrees
Relative target bearing to the tree = 217.519650 degrees

******************************************************************************/

/*
Configuration settings.

These defines makes it easy for you to enable/disable certain
code during the development and debugging cycle of this project.
There may not be sufficient room in the PROGRAM or DATA memory to
enable all these libraries at the same time.  You must have NEO_ON,
GPS_ON and SDC_ON during the actual GeoCache Flag Hunt on Finals Day.
*/
#define NEO_ON 1		// NeoPixelShield
#define TRM_ON 1		// SerialTerminal
#define SDC_ON 1		// SecureDigital
#define GPS_ON 0		// Live GPS Message (off = simulated)

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive
#define Brightness A0
#define FLAGSELECT 2
#define FRAME_TIME 20

struct GPSMessage
{
	int time;
	bool isValid;
	bool north;
	bool east;
	double latitude;
	double longitude;
	uint16_t SOGKnots;
	float COGDegrees;
	int date;
	float MagVar;
};

// GPS message buffer
#define GPS_RX_BUFSIZ	128
char cstr[GPS_RX_BUFSIZ];

// global variables
uint8_t target = 0;		// target number
float heading = 0.0;	// target heading
float distance = 0.0;	// target distance
GPSMessage message;
#define PITCH 8


#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if SDC_ON
#include <SD.h>

SDLib::File file;
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.

On GeoCache day, you will be given waypoints in Decimal Degrees format for 4x
flags located on Full Sail campus.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#endif

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *cind, char *ccor)

Input:
cind = char string pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
ccor = char string pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate

Return:
Decimal degrees coordinate.

**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	float decimalDegrees = 0.0;
	float degrees = ccor[0] * 10 + ccor[1];
	float minutes = ccor[2] * 10 + ccor[3] + ccor[5] * 0.1f + ccor[6] * 0.01f + ccor[7] * 0.001f + ccor[8] * 0.0001f;



	return(decimalDegrees);
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
flat1, flon1 = course over ground latitude and longitude coordinate in decimal degrees
flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	float distance = 0.0;
	float dLon = abs(flon1 - flon2);
	float dLat = abs(flat1 - flat2);
	float a = (sin(dLat / 2)) * (sin(dLat / 2)) + cos(flat1) * cos(flat2) * (sin(dLon / 2)) * (sin(dLon / 2));
	float c = 2 * atan2(sqrt(a), sqrt(1 - a));
	// add code here

	return(c * 3959 * 5280);
}

/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
flat1, flon1 = course over ground latitude and longitude coordinate in decimal degrees
flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
angle in decimal degrees from magnetic north (NOTE: arc tangent returns range of -pi/2 to +pi/2)
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	return(atan2(sin(flon1 - flon2) * cos(flat2), cos(flat1) * sin(flat2) - sin(flat1) * cos(flon1 - flon2)));
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/

#if NEO_ON
/*
Sets target number, heading and distance on NeoPixel Display

NOTE: Target number, bearing and distance parameters used
by this function do not need to be passed in, since these
parameters are in global data space.

*/
void setNeoPixel(int target, int heading, int distance)
{
	//Set the target
	drawNumber(target);
	//display distance
	drawDistance(distance);
	//calculate arrow to show based off heading

	// If on top of thing, print that it's here
	if (distance < 15)
	{
		drawArrow(0);
		return;
	}

	// TODO: If stopped, draw the X
	if (false)
	{
		drawArrow(10);
		return;
	}

	// Draw the correct arrow otherwise
	drawArrow(1 + map(heading, 0, 360, 0, 9));

}

#endif	// NEO_ON

#if GPS_ON
/*
Get valid GPS message. This function returns ONLY once a second.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Message is placed in global "cstr" string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	uint8_t x = 0, y = 0, isum = 0;

	memset(cstr, 0, sizeof(cstr));

	// get nmea string
	while (true)
	{
		if (gps.peek() != -1)
		{
			cstr[x] = gps.read();

			// if multiple inline messages, then restart
			if ((x != 0) && (cstr[x] == '$'))
			{
				x = 0;
				cstr[x] = '$';
			}

			// if complete message
			if ((cstr[0] == '$') && (cstr[x++] == '\n'))
			{
				// nul terminate string before /r/n
				cstr[x - 2] = 0;

				// if checksum not found
				if (cstr[x - 5] != '*')
				{
					x = 0;
					continue;
				}

				// convert hex checksum to binary
				isum = strtol(&cstr[x - 4], NULL, 16);

				// reverse checksum
				for (y = 1; y < (x - 5); y++) isum ^= cstr[y];

				// if invalid checksum
				if (isum != 0)
				{
					x = 0;
					continue;
				}

				// else valid message
				break;
			}
		}
	}
}

#else
/*
Get simulated GPS message once a second.

This is the same message and coordinates as described at the top of this
file.  You could edit these coordinates to point to the tree out front (GEOLAT0,
GEOLON0) to test your distance and direction calculations.  Just note that the
tree coordinates are in Decimal Degrees format, and the message coordinates are
in Degrees Minutes format.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Static GPRMC message is placed in global "cstr" null terminated char string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	static unsigned long gpsTime = 0;

	// simulate waiting for message
	while (gpsTime > millis()) delay(100);

	// do this once a second
	gpsTime = millis() + 1000;

	memcpy(cstr, "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));

	return;
}

#endif	// GPS_ON


///draw arrow
void drawArrow(uint8_t direction)
{
	//draw forward arrow
	if (direction == 1)
	{
		setPixelColor(5, 0, 0, 0, 255);
		setPixelColor(6, 0, 0, 0, 255);
		setPixelColor(4, 1, 0, 0, 255);
		setPixelColor(5, 1, 0, 0, 255);
		setPixelColor(6, 1, 0, 0, 255);
		setPixelColor(7, 1, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(5, 3, 0, 0, 255);
		setPixelColor(6, 3, 0, 0, 255);
		setPixelColor(5, 4, 0, 0, 255);
		setPixelColor(6, 4, 0, 0, 255);
	}
	//draw forward/right arrow
	if (direction == 2)
	{
		setPixelColor(4, 0, 0, 0, 255);
		setPixelColor(5, 0, 0, 0, 255);
		setPixelColor(6, 0, 0, 0, 255);
		setPixelColor(7, 0, 0, 0, 255);
		setPixelColor(6, 1, 0, 0, 255);
		setPixelColor(7, 1, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(7, 2, 0, 0, 255);
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(7, 3, 0, 0, 255);
	}
	//draw right arrow
	if (direction == 3)
	{
		setPixelColor(5, 0, 0, 0, 255);
		setPixelColor(6, 1, 0, 0, 255);
		setPixelColor(4, 2, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(7, 2, 0, 0, 255);
		setPixelColor(6, 3, 0, 0, 255);
		setPixelColor(5, 4, 0, 0, 255);
	}
	//draw back right arrow
	if (direction == 4)
	{
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(5, 3, 0, 0, 255);
		setPixelColor(6, 3, 0, 0, 255);
		setPixelColor(7, 3, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(7, 2, 0, 0, 255);
		setPixelColor(5, 1, 0, 0, 255);
		setPixelColor(7, 1, 0, 0, 255);
		setPixelColor(4, 0, 0, 0, 255);
		setPixelColor(7, 0, 0, 0, 255);
	}
	//draw down arrow
	if (direction == 5)
	{
		setPixelColor(5, 0, 0, 0, 255);
		setPixelColor(6, 0, 0, 0, 255);
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(5, 1, 0, 0, 255);
		setPixelColor(6, 1, 0, 0, 255);
		setPixelColor(7, 3, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(5, 3, 0, 0, 255);
		setPixelColor(6, 3, 0, 0, 255);
		setPixelColor(5, 4, 0, 0, 255);
		setPixelColor(6, 4, 0, 0, 255);
	}
	//drw down left arrow
	if (direction == 6)
	{
		setPixelColor(7, 3, 0, 0, 255);
		setPixelColor(6, 3, 0, 0, 255);
		setPixelColor(5, 3, 0, 0, 255);
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(4, 2, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(4, 1, 0, 0, 255);
		setPixelColor(6, 1, 0, 0, 255);
		setPixelColor(4, 0, 0, 0, 255);
		setPixelColor(7, 0, 0, 0, 255);
	}
	//draw left arrow
	if (direction == 7)
	{
		setPixelColor(6, 0, 0, 0, 255);
		setPixelColor(5, 1, 0, 0, 255);
		setPixelColor(4, 2, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(7, 2, 0, 0, 255);
		setPixelColor(5, 3, 0, 0, 255);
		setPixelColor(6, 4, 0, 0, 255);


	}
	//draw up/left arrow
	if (direction == 8)
	{
		setPixelColor(4, 0, 0, 0, 255);
		setPixelColor(5, 0, 0, 0, 255);
		setPixelColor(6, 0, 0, 0, 255);
		setPixelColor(7, 0, 0, 0, 255);
		setPixelColor(4, 1, 0, 0, 255);
		setPixelColor(5, 1, 0, 0, 255);
		setPixelColor(4, 2, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(7, 3, 0, 0, 255);
	}
	//draw centered cirlce
	if (direction == 9)
	{
		setPixelColor(5, 0, 0, 0, 255);
		setPixelColor(6, 0, 0, 0, 255);
		setPixelColor(4, 1, 0, 0, 255);
		setPixelColor(7, 1, 0, 0, 255);
		setPixelColor(4, 2, 0, 0, 255);
		setPixelColor(7, 2, 0, 0, 255);
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(7, 3, 0, 0, 255);
		setPixelColor(5, 4, 0, 0, 255);
		setPixelColor(6, 4, 0, 0, 255);
	}
	//draw not moving x
	if (direction == 10)
	{
		setPixelColor(4, 0, 0, 0, 255);
		setPixelColor(7, 0, 0, 0, 255);
		setPixelColor(6, 1, 0, 0, 255);
		setPixelColor(5, 1, 0, 0, 255);
		setPixelColor(5, 2, 0, 0, 255);
		setPixelColor(6, 2, 0, 0, 255);
		setPixelColor(4, 3, 0, 0, 255);
		setPixelColor(7, 3, 0, 0, 255);
	}
}

void print()
{
	strip.show();
	clearScreen();
}

void writeToSD(uint16_t bearing, uint32_t distance)
#if SDC_ON
{
	char line[40];
	char lat[16];
	char lng[16];
	dtostrf(message.latitude, 8, 6, lat);
	dtostrf(message.longitude, 8, 6, lng);
	sprintf(line, "%s,%s,%d.%d\n", lat, lng, bearing, distance);
	file.write(line, strlen(line));
}
#elif
{
	// Intentional stub
}
#endif

void setup(void)
{
	pinMode(Brightness, INPUT);
	pinMode(FLAGSELECT, INPUT_PULLUP);

#if TRM_ON
	// init serial interface
	Serial.begin(115200);
#endif	

#if NEO_ON
	strip.begin();
	strip.show();
#endif	


#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapNN.txt" for storing your coordinates, where NN is the
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").
	*/
	SD.begin();

	char *fileName = "MyMap0/.txt";
	
	for (int i = 0; i < 100; ++i)
	{
		if (i > 0 && i % 10 == 0)
		{
			++fileName[5];
			fileName[6] = '/';
		}
		++fileName[6];
		if (!SD.exists(fileName))
		{
			file = SD.open(fileName);
			break;
		}
	}
#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// init target button here
}

void loop(void)
{
	unsigned long currentTime = millis();
	// max 1 second blocking call till GPS message received
	getGPSMessage();

#if TRM_ON
//	Serial.println(cstr);
#endif	

	// if button pressed, set new target

	// if GPRMC message (3rd letter = R)
	while (cstr[3] == 'R')
	{
		// parse message parameters

		// calculated destination heading

		// calculated destination distance

#if SDC_ON
		// write current position to SecureDigital then flush
//		writeToSD(heading, distance);
//		file.flush();
#endif

		break;
	}

#if NEO_ON
	// set NeoPixel target display
//	setNeoPixel(target, heading, distance);

	static unsigned long timestamp = 0;
	//print to the neo pixel if the time has expired
	strip.setBrightness(analogRead(Brightness) / 4);

	setNeoPixel(random(0, 10), random(0, 360), random(0, 550));

	print();

#endif	
}

#define RED 255, 0, 0
#define GREEN 0, 255, 0
#define BLUE 0, 0, 255
#define YELLOW 255, 255, 0
#define CYAN 0, 255, 255
#define MAGENTA 255, 0, 255
#define WHITE 255, 255, 255
#define BLACK 0, 0, 0

void clearScreen()
{
	for (uint8_t x = 0; x < PITCH; ++x)
	{
		for (uint8_t y = 0; y < 5; ++y)
		{
			setPixelColor(x, y, BLACK);
		}
	}
}

void setPixelColor(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t index = (y * PITCH) + x;
	
	strip.setPixelColor(index, strip.Color(r, g, b));
}


#pragma region DrawFlagNumber

// 000   Top row is 0
// 0 0   Left is 3, Right is 4
// 000   Middle row is 1
// 0 0   Left is 5, Right is 6
// 000   Bottom row is 2

#define	ZERO	125	
#define	ONE		80
#define	TWO		55
#define	THREE	87
#define	FOUR	90
#define	FIVE	79
#define SIX		111
#define SEVEN	81
#define	EIGHT	127
#define	NINE	95

void drawNumber(uint8_t n)
{
	uint8_t instructions = 0;
	switch (n)
	{
	case 0:
		instructions = ZERO;
		break;
	case 1:
		instructions = ONE;
		break;
	case 2:
		instructions = TWO;
		break;
	case 3:
		instructions = THREE;
		break;
	case 4:
		instructions = FOUR;
		break;
	case 5:
		instructions = FIVE;
		break;
	case 6:
		instructions = SIX;
		break;
	case 7:
		instructions = SEVEN;
		break;
	case 8:
		instructions = EIGHT;
		break;
	case 9:
		instructions = NINE;
		break;
	}

	for (uint8_t i = 0; i < 7; ++i)
	{
		if ((1 << i) & instructions)
		{
			switch (i)
			{
			case 0:
				setPixelColor(0, 0, RED);
				setPixelColor(1, 0, RED);
				setPixelColor(2, 0, RED);
				break;
			case 1:
				setPixelColor(0, 2, RED);
				setPixelColor(1, 2, RED);
				setPixelColor(2, 2, RED);
				break;
			case 2:
				setPixelColor(0, 4, RED);
				setPixelColor(1, 4, RED);
				setPixelColor(2, 4, RED);
				break;
			case 3:
				setPixelColor(0, 0, RED);
				setPixelColor(0, 1, RED);
				setPixelColor(0, 2, RED);
				break;
			case 4:
				setPixelColor(2, 0, RED);
				setPixelColor(2, 1, RED);
				setPixelColor(2, 2, RED);
				break;
			case 5:
				setPixelColor(0, 2, RED);
				setPixelColor(0, 3, RED);
				setPixelColor(0, 4, RED);
				break;
			case 6:
				setPixelColor(2, 2, RED);
				setPixelColor(2, 3, RED);
				setPixelColor(2, 4, RED);
				break;
			}
		}
	}
}

#pragma endregion

#pragma region DrawDistance

#define DIST_1 15
#define DIST_2 30
#define DIST_3 45
#define DIST_4 60
#define DIST_5 75

#define DIST_6 100
#define DIST_7 150
#define DIST_8 200
#define DIST_9 250
#define DIST_10 300

#define DIST_11 400
#define DIST_12 500
#define DIST_13 600
#define DIST_14 700
#define DIST_15 800

#define DIST_16 1000
#define DIST_17 2000
#define DIST_18 5000
#define DIST_19 10000
#define DIST_20 50000

void drawDistance(uint32_t d)
{
	if (d > DIST_1)
		setPixelColor(3, 4, GREEN);
	if (d > DIST_2)
		setPixelColor(3, 3, GREEN);
	if (d > DIST_3)
		setPixelColor(3, 2, GREEN);
	if (d > DIST_4)
		setPixelColor(3, 1, GREEN);
	if (d > DIST_5)
		setPixelColor(3, 0, GREEN);
	if (d > DIST_6)
		setPixelColor(3, 4, YELLOW);
	if (d > DIST_7)
		setPixelColor(3, 3, YELLOW);
	if (d > DIST_8)
		setPixelColor(3, 2, YELLOW);
	if (d > DIST_9)
		setPixelColor(3, 1, YELLOW);
	if (d > DIST_10)
		setPixelColor(3, 0, YELLOW);
	if (d > DIST_11)
		setPixelColor(3, 4, RED);
	if (d > DIST_12)
		setPixelColor(3, 3, RED);
	if (d > DIST_13)
		setPixelColor(3, 2, RED);
	if (d > DIST_14)
		setPixelColor(3, 1, RED);
	if (d > DIST_15)
		setPixelColor(3, 0, RED);
	if (d > DIST_16)
		setPixelColor(3, 4, WHITE);
	if (d > DIST_17)
		setPixelColor(3, 3, WHITE);
	if (d > DIST_18)
		setPixelColor(3, 2, WHITE);
	if (d > DIST_19)
		setPixelColor(3, 1, WHITE);
	if (d > DIST_20)
		setPixelColor(3, 0, WHITE);
#pragma endregion
}