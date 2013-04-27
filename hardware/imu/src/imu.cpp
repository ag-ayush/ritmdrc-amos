// CMPS03 Interface Code
// Simply spit out compass data in the following format:
// 	[STX:0x02] [HI-byte] [LO-byte]
// Compile it using Arduino

#define AVG 0

#include <WProgram.h>

#include <Wire.h>

#include <ros.h>
#include <imu_msg/Orientation.h>
#include <std_msgs/Empty.h> //Used for calibration messages

#define AVG_WEIGHT 0.5f /* How much of the new reading we take */

#define I2C_SLAVE_ADDR 0x42

#define CMPS03_ADDR 0x60

#define IMU_PACKET_LEN 14
#define IMU_HEAD 4
#define IMU_TAIL 2

#define BUF_LEN (IMU_PACKET_LEN + 1)

#define ToRad(x) ((x)*0.01745329252)

#define I_PI_2 1.570796
#define I_PI   3.141592
#define I_PI2  6.283184

#define DEBUG_SERIAL 0

imu_msg::Orientation orientation;

#if DEBUG_SERIAL != 1
ros::NodeHandle nh;
ros::Publisher pImu("imu", &orientation);

void calibration(const std_msgs::Empty& emsg) {
	Wire.beginTransmission(CMPS03_ADDR);
	Wire.send(15);
	Wire.send(0xFF);
	Wire.endTransmission();
}
ros::Subscriber<std_msgs::Empty> sCal("cmps03_cal", calibration);
#endif

//Temporary variables that we read from the IMU
// before converting them and sending off to ROS
//Each is a 2 byte number with 1/100 precision
int tRoll, tPitch, tYaw;
uint8_t i2c_buf[BUF_LEN];
int start = 0, end = 0, newData = 0;
unsigned char chka, chkb;

void imuHandler(int num) {
	#if DEBUG_SERIAL == 1
		//Serial.println(num);
	#endif
	
	if (num == 14) {
		newData = 1;
		for (int i = 0; i < num; ++i) {
			i2c_buf[i] = Wire.receive();
		}
	} else {
		while (Wire.available()) Wire.receive();
	}
}

void processBuf() {
	if (i2c_buf[0] == 'M' && i2c_buf[1] == 'D' && i2c_buf[2] == 'R' && i2c_buf[3] == 'C' &&
			i2c_buf[IMU_PACKET_LEN - 1] == '!' && i2c_buf[IMU_PACKET_LEN - 2] == '!') {
			
		chka = 0;
		chkb = 0;
		
		for (int i = 0; i < IMU_PACKET_LEN - 4; ++i) {
			chka += i2c_buf[i];
			chkb += chka;
		}
		
		if (chka == i2c_buf[IMU_PACKET_LEN - 4] && chkb == i2c_buf[IMU_PACKET_LEN - 3]) {
			int a = 4;
			
			int t;
			t = i2c_buf[a++] << 8;
			t |= i2c_buf[a++];
			orientation.roll = ToRad(t / 100.0);
			
			t = i2c_buf[a++] << 8;
			t = i2c_buf[a++];
			orientation.pitch = ToRad(t / 100.0);
			
			t = i2c_buf[a++] << 8;
			t += i2c_buf[a++];
            double cmps = ToRad(90 - t / 100.0);
			//orientation.heading = AVG_WEIGHT * cmps + (1.0 - AVG_WEIGHT) * orientation.heading;
            orientation.heading = cmps;
			
			while (orientation.heading < -I_PI) orientation.heading += I_PI2;
			while (orientation.heading > I_PI) orientation.heading -= I_PI2;
			
			#if DEBUG_SERIAL == 1
				Serial.print("Orientation: (");
				Serial.print(orientation.roll, DEC);
				Serial.print(", ");
				Serial.print(orientation.pitch, DEC);
				Serial.print(", ");
				Serial.print(orientation.heading, DEC);
				Serial.println(")");
			#else
				pImu.publish(&orientation);
			#endif
		}
		
		#if DEBUG_SERIAL == 1
			else {
				Serial.print("Checksum mismatch! Calculated (");
				Serial.print(chka, DEC);
				Serial.print(", ");
				Serial.print(chkb, DEC);
				Serial.print(") Got (");
				Serial.print(i2c_buf[10], DEC);
				Serial.print(", ");
				Serial.print(i2c_buf[11], DEC);
				Serial.println(")");
			}
		#endif
	}
	
	#if DEBUG_SERIAL == 1
		else {
			Serial.print("Header mismatch! Got: '");
			for (int i = 0; i < 4; ++i) {
				Serial.print(i2c_buf[i]);
			}
			Serial.println("'");
		}
	#endif
}

void setup()
{
	//Initialize I2C and serial
	//This will now be an I2C slave. The IMU
	// will push I2C data to us, which we can then publish
	// to ROS.
	Wire.begin(I2C_SLAVE_ADDR);
	Wire.onReceive(imuHandler);
	
	//pinMode(LED_PIN, OUTPUT);

	#if DEBUG_SERIAL == 1
		Serial.begin(38400);
	#else
		nh.initNode();
		nh.advertise(pImu);
		nh.subscribe(sCal);
	#endif
}

void loop()
{
    if (newData) {
    	#if DEBUG_SERIAL == 1
    		Serial.println("Processing new data");
    	#endif
    	
    	newData = 0;
    	processBuf();
    }
    
    #if DEBUG_SERIAL == 1
    	Serial.println("Waiting...");
    	delay(500);
    #else
	    nh.spinOnce();
	#endif
}
