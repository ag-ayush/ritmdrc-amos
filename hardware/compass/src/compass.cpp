// CMPS03 Interface Code
// Simply spit out compass data in the following format:
// 	[STX:0x02] [HI-byte] [LO-byte]
// Compile it using Arduino

#define AVG 0

#if ARDUINO>=100
#include <Arduino.h>
#define send write
#define receive read
#else
#include <WProgram.h>
#endif
#include <Wire.h>

#include <ros.h>
#include <compass_msg/Compass.h>

#define POLL_TIME 10
#define REPORT_INTERVAL 5
#define BAUD_RATE 115200
#define LED_PIN 13
#define RUNNING_AVG_WEIGHT 0.2f


ros::NodeHandle nh;

compass_msg::Compass compassMsg;
ros::Publisher compass_pub("compass", &compassMsg);


void setup()
{
	//Initialize I2C and serial
	Wire.begin();
	//pinMode(LED_PIN, OUTPUT);

    nh.initNode();
    nh.advertise(compass_pub);
}

uint16_t heading = 0;
uint16_t i = 0;

void loop()
{
	// request transfer from register 1
	Wire.beginTransmission(96);
	Wire.send(2);
	Wire.endTransmission();

	// read register
	Wire.requestFrom(96, 2);
	/*byte hi = 0, low = 0;
	if (Wire.available())
		hi = Wire.receive();
	if (Wire.available())
		low = Wire.receive();*/

	uint16_t value = 0;//(uint16_t)(hi << 8 | low);
	if (Wire.available() >= 2) {
  	value = Wire.receive() << 8;
  	value |= Wire.receive();
  }
	// running average
	heading = RUNNING_AVG_WEIGHT * value + (1.0f - RUNNING_AVG_WEIGHT) * heading;

	// blink LED

	// report
	if (!(++i % REPORT_INTERVAL))
	{
		i = 0;
		compassMsg.heading = heading / 10.0;
		compass_pub.publish(&compassMsg);
	}
    delay(20);
    nh.spinOnce();
}
