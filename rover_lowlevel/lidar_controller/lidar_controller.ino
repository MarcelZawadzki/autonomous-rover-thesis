//libs:
#include <Servo.h>
#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>
#include <string.h>

#define step_sleep 4
#define step_stp 5
#define step_dir 6
#define step_mid 7
#define svlid 10

//variables
LIDARLite_v3HP myLidarLite;
#define FAST_I2C

Servo servolidar;

struct LidarParams{
	LidarParams(int u_step_multiplier=1, int v_res=5, int v_min_angle=70, int v_step=10):
				svlidmin(v_min_angle),
				svlidh(v_step),
				svlidmax( svlidmin + v_step * (v_res-1) ),
				steps_per_rev(200*u_step_multiplier),
				lid_i_max((uint32_t)steps_per_rev * (uint32_t)v_res),
				switch_time(100){	//measured to be equal to time of execution of lidar_level()
		svlidp = svlidmin;
		lid_i = 0;
		iflidar=0;
		svlid_dir = 1;
		initialized = 0;
	}
	const int svlidmin, svlidmax, svlidh, steps_per_rev;
	const uint32_t lid_i_max;
	int svlidp;
	uint32_t lid_i;
	bool iflidar, svlid_dir;
	uint16_t dist;
	bool initialized;
	const uint32_t switch_time;

    // void toString(char* buffer, size_t size) const {
    //     snprintf(buffer, size,
    //              "svlidmin: %d, svlidmax: %d, svlidh: %d, "
    //              "steps_per_rev: %d, lid_i_max: %lu, "
    //              "svlidp: %d, lid_i: %lu, iflidar: %d, "
    //              "svlid_dir: %d, dist: %u, initialized: %d",
    //              svlidmin, svlidmax, svlidh,
    //              steps_per_rev, lid_i_max,
    //              svlidp, lid_i, iflidar,
    //              svlid_dir, dist, initialized);
    // }
};

LidarParams* lidar_params = new LidarParams();	//default preset



void setup()
{
	pinMode(step_sleep, OUTPUT);
	pinMode(step_stp, OUTPUT);
	pinMode(step_dir, OUTPUT);
	pinMode(step_mid, INPUT_PULLUP);

	servolidar.attach(svlid);
	servo_test();

	Serial.begin(115200);

	Wire.begin();

	#ifdef FAST_I2C
			#if ARDUINO >= 157
				Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
			#else
				TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
			#endif
	#endif

	myLidarLite.configure(8);     //LIDAR	Modified Library!!!

	delay(250);					  //Prevent error after lidar configuration

	digitalWrite(step_sleep, LOW); //disable step driver
}


//Compatibility with multiple means of servo control
void servoset(float p){
    //  servolidar.write(750+p*1500.0/180.0);
	int angle = p + 2;
	if (angle < 40) angle = 40;
	if (angle > 140) angle = 140;
    servolidar.write(angle);
}



/////////////////////
//Stepper functions//
/////////////////////


void step_initialize()
{
    digitalWrite(step_sleep, HIGH); //enable step driver
	int cal_delay = 8000;
    digitalWrite(step_dir, LOW);
    if(digitalRead(step_mid)==LOW)
    {
        while(digitalRead(step_mid)==LOW)
        {
            digitalWrite(step_stp, HIGH);
            delayMicroseconds(cal_delay);
            digitalWrite(step_stp, LOW);
            delayMicroseconds(cal_delay);
        }
    }
    while(digitalRead(step_mid)==HIGH)
    {
        digitalWrite(step_stp, HIGH);
        delayMicroseconds(cal_delay);
        digitalWrite(step_stp, LOW);
        delayMicroseconds(cal_delay);
    }
	int calibrate_assist=0;
    while(digitalRead(step_mid)==LOW)
    {
        digitalWrite(step_stp, HIGH);
        delayMicroseconds(cal_delay);
        digitalWrite(step_stp, LOW);
        delayMicroseconds(cal_delay);
        calibrate_assist++;
    }
    digitalWrite(step_dir, HIGH);
    for(int i=0;i<=calibrate_assist/2;i++)
    {
        digitalWrite(step_stp, HIGH);
        delayMicroseconds(cal_delay);
        digitalWrite(step_stp, LOW);
        delayMicroseconds(cal_delay);
    }
  //    digitalWrite(step_dir, LOW);
    delay(1000);
}


void step_disable()
{
    digitalWrite(step_sleep, LOW); //enable step driver
}


// benchamrk
// void step_test()
// {
// 	//4700,700,4x
// 	digitalWrite(step_dir, HIGH);
// 	for(int i=0;i<1000;i++)
// 	{
// 		digitalWrite(step_stp, HIGH);
// 		delayMicroseconds(4500-4*i);
// 		digitalWrite(step_stp, LOW);
// 		delayMicroseconds(4500-4*i);
// 	}
// 	for(int i=0;i<600;i++)
// 	{
// 		digitalWrite(step_stp, HIGH);
// 		delayMicroseconds(500);
// 		digitalWrite(step_stp, LOW);
// 		delayMicroseconds(500);
// 	}
// 	for(int i=0;i<1000;i++)
// 	{
// 		digitalWrite(step_stp, HIGH);
// 		delayMicroseconds(500+4*i);
// 		digitalWrite(step_stp, LOW);
// 		delayMicroseconds(500+4*i);
// 	}

// 	delay(2000);
// }


void servo_test(){
	servoset(90);
	delay(500);
	servoset(60);
	delay(500);
	servoset(120);
	delay(500);
	servoset(90);
	delay(500);
}



///////////////////
//Lidar functions//
///////////////////


void lidar_scan()
{

    /*
    * preset1:    preset2:
    * i = 1400    i = 2000
    *   min j_li = 50
    *   max j_li:
    * 900         1100
    * lidar delays:
    * 250, 175    250, 50
    */
    if(lidar_params->lid_i>=lidar_params->lid_i_max) {
		lidar_params->iflidar=0;
    }
    else {
		// // //Normal mode
		// if(lidar_params->lid_i%lidar_params->steps_per_rev==0) lidar_level();
		// myLidarLite.waitForBusy();
		// myLidarLite.takeRange();
		// myLidarLite.waitForBusy();						//fast mode off
		// digitalWrite(step_stp, HIGH);
		// lidar_params->dist = myLidarLite.readDistance();
		// Serial.println(lidar_params->dist);
		// delayMicroseconds(250);
		// digitalWrite(step_stp, LOW);
		// delayMicroseconds(150);
		// lidar_params->lid_i++;

		// //S mode
		// if(lidar_params->lid_i%lidar_params->steps_per_rev==0) lidar_level();
		// myLidarLite.waitForBusy();
		// myLidarLite.takeRange();
		// digitalWrite(step_stp, HIGH);
		// myLidarLite.waitForBusy();
		// lidar_params->dist = myLidarLite.readDistance();
		// Serial.println(lidar_params->dist);
		// // delayMicroseconds(250);
		// digitalWrite(step_stp, LOW);
		// // delayMicroseconds(150);
		// lidar_params->lid_i++;

		//SS mode
		if(lidar_params->lid_i%lidar_params->steps_per_rev==0){
			lidar_level();
		}
		else{
			delayMicroseconds(lidar_params->switch_time);
		}
		myLidarLite.waitForBusy();
		myLidarLite.takeRange();
		lidar_params->dist = myLidarLite.readDistance();	//reads old distance

		digitalWrite(step_stp, HIGH);

		// speed equalizing block
		if (lidar_params->dist < 10000){
			delayMicroseconds(40);
			if (lidar_params->dist < 1000){
				delayMicroseconds(40);
				if (lidar_params->dist < 100){
					delayMicroseconds(40);
					if (lidar_params->dist < 10){
						delayMicroseconds(40);
					}
				}
			}
		}
		Serial.println(lidar_params->dist);
		lidar_params->lid_i++;

		digitalWrite(step_stp, LOW);
    }
}

void lidar_level()
{
    Serial.println("N");
    if(lidar_params->svlid_dir)
    lidar_params->svlidp+=lidar_params->svlidh;
    else
    lidar_params->svlidp-=lidar_params->svlidh;

    servoset(180-lidar_params->svlidp);
}


void lidar_new_scan()
{
	lidar_params->lid_i = lidar_params->steps_per_rev;

	lidar_params->svlid_dir = !lidar_params->svlid_dir;

	if(lidar_params->svlid_dir) {
		lidar_params->svlidp=lidar_params->svlidmin;
		Serial.println("B");
	}
	else {
		lidar_params->svlidp=lidar_params->svlidmax;
		Serial.println("T");
	}
}


void lidar_ini()
{
	step_initialize();
	lidar_calibrate();
	lidar_params->svlidp=lidar_params->svlidmin-lidar_params->svlidh;
	lidar_params->lid_i=0;
	servoset(180-lidar_params->svlidmin);
	delay(1000);

	//accelerate
	int acc_steps = lidar_params->steps_per_rev/2+1600;
	for(uint32_t i=0;i<acc_steps;i++) {
		uint32_t j_li=50+((i*1000) / acc_steps);
		uint32_t k_li=500000/j_li;
		//Serial.println(k_li);
		digitalWrite(step_stp, HIGH);
		//delayMicroseconds(3600-5*i);
		delayMicroseconds(k_li);
		digitalWrite(step_stp, LOW);
		delayMicroseconds(k_li);
		//delayMicroseconds(3600-5*i);
	}
	lidar_params->initialized = 1;
	Serial.println("I");
}


void lidar_disable()
{
	step_disable();
	servoset(90);
	lidar_params->initialized = 0;
}


void lidar_calibrate()
{
    for(int i=0;i<1000;i++)
    {
		myLidarLite.waitForBusy();
        myLidarLite.takeRange();
        myLidarLite.waitForBusy();
        myLidarLite.readDistance();
    }
}


void loop()
{
	if (Serial.available() > 0)	{
		char cmd = Serial.read();
		switch(cmd){
			case 'I': {
				int u_step_multiplier = Serial.parseInt();
				int v_res = Serial.parseInt();
				int v_min_angle = Serial.parseInt();
				int v_step = Serial.parseInt();
				delete lidar_params;
				lidar_params = new LidarParams(u_step_multiplier, v_res, v_min_angle, v_step);

				// feedback
				// const size_t bufferSize = 256;
				// char buffer[bufferSize];
				// lidar_params->toString(buffer, bufferSize);
				// Serial.println(buffer);

				lidar_ini();
				break;
			}
			case 'S':
				lidar_disable();
				break;
		}
	}
	if (lidar_params->initialized) {
		lidar_params->iflidar = 1;
		while(lidar_params->iflidar) lidar_scan();
		lidar_new_scan();
    }
}
