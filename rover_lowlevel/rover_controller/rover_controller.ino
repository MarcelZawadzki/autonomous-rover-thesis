//libs:
#include <DHT.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include <stdint.h>
#include <Wire.h>


//defs:
#define in1 25 //ZAMIENIONE!!! NIE ZGODNE Z WPIECIEM W ARDUINO ALE NIE CHCE ZMIENIAC KODU
#define in2 24 //ZAMIENIONE!!! NIE ZGODNE Z WPIECIEM W ARDUINO ALE NIE CHCE ZMIENIAC KODU
#define in3 23 //ZAMIENIONE!!! NIE ZGODNE Z WPIECIEM W ARDUINO ALE NIE CHCE ZMIENIAC KODU
#define in4 22 //ZAMIENIONE!!! NIE ZGODNE Z WPIECIEM W ARDUINO ALE NIE CHCE ZMIENIAC KODU
#define ena 7 //ZAMIENIONE!!! NIE ZGODNE Z WPIECIEM W ARDUINO ALE NIE CHCE ZMIENIAC KODU
#define enb 6 //ZAMIENIONE!!! NIE ZGODNE Z WPIECIEM W ARDUINO ALE NIE CHCE ZMIENIAC KODU
#define svs 4
//#define d11 2
//#define bts 26
#define current A0
#define voltage A1
#define statled 27
#define headled 35
#define svpan 30
#define svtilt 29
#define lsr 28

#define modeSW 52

//global variables:
unsigned long time_m=0;
unsigned long time_m_upd=0;
unsigned long time_m_upd2=0;
unsigned long time_m_upd3=0;
bool authorisation=false;
bool skip=0;

//power
const int lspeed=160;
const int qtest=10;

char rx_cmd;//string to store app command state.
char rx_cmd3;//string to store rpi command state.
int button; //  Store how many times a button has been pressed.
//bool BTstate=0;

int speed=0;
int SpdR=0;
int SpdL=0;
bool powerlock=1;

int tankspeed=0;
bool is_tank=0;

//int tempspd=0;                         //był float

//sensors

int mq2_analog=0;
int mq2_map=0;
int mq2_map2=0;

int temp=0;
int humid=0;

int amp_tab[qtest];
uint32_t amp_i=0;
int amp_res=0;
int amp_analog=0;
int amp_map=0;
int ampcalibrate=-60;   //-100;

int volt_analog=0;
int volt=0;
float volt_res=0;
int volt_zero=1800;//volt*100

bool laser_status=0;
bool lighter_status=0;
int status_bright=100;
const uint32_t head_bright=180;
uint32_t head_bright_2;

//activate

//DHT dht11;

//servos

Servo serwomechanizm;
int steer_angle = 90;

Servo servopan;
int pan_zero=84;
int pan_pos = pan_zero;
bool acs=0;
Servo servotilt;
int tilt_zero=110;
int tilt_pos = tilt_zero;


Adafruit_NeoPixel stat_led = Adafruit_NeoPixel(1, statled, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel head_led = Adafruit_NeoPixel(6, headled, NEO_RGB + NEO_KHZ800);
extern const uint8_t gamma8[];

//Mode: Autonomous or manual
bool mode=0;

//GPS
static NMEAGPS gps;
gps_fix fix;
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
const PROGMEM  uint8_t Navrate4hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};

const PROGMEM  uint8_t EnableGST[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x0B, 0x68};
const PROGMEM  uint8_t DisableVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
const PROGMEM  uint8_t DisableGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
const PROGMEM  uint8_t DisableGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x62};
const PROGMEM  uint8_t DisableGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};

static const NeoGPS::Location_t Base(525897720, 214486120); //(52.589772, 21.448612)


//9dof
MPU9250 imu;
I2Cdev I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float pitch;
float roll;

float pitch_center=-0.02;
float roll_center=-0.06;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 35;
static float my_centre = 3;
static float mz_centre = 21;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;




void setup() {
	stat_led.begin();
	stat_led.clear();
	stat_led.show();

	head_led.begin();
	head_led.clear();
	head_led.show();

	head_bright_2=lights_ini();

	//flash_on(0,status_bright,0,3);

	// moto driver
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	pinMode(ena, OUTPUT);
	pinMode(enb, OUTPUT);

	for(int i=0; i<qtest; i++)
	amp_tab[i]=0;

	//dht11.setup(d11);

	serwomechanizm.attach(svs);

	servopan.attach(svpan);
	servotilt.attach(svtilt);

	pinMode(lsr, OUTPUT);
	digitalWrite (lsr,LOW);
	CAM();

	pinMode(modeSW, INPUT_PULLUP);

	Serial1.begin(2400);		//Radio connection
	Serial1.setTimeout(200);  //170
	gpsPort.begin(9600);		//GPS connection
	GPS_SendConfig(Navrate4hz, 14);
	GPS_SendConfig(EnableGST, 16);
	GPS_SendConfig(DisableVTG, 16);
	GPS_SendConfig(DisableGSV, 16);
	GPS_SendConfig(DisableGLL, 16);
	GPS_SendConfig(DisableGSA, 16);
	//DEBUG_PORT.begin(9600);
	Serial.begin(115200);		//PC connection
	Serial.setTimeout(5);


	Wire.begin();

	#ifdef FAST_I2C
		#if ARDUINO >= 157
			Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
		#else
			TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
		#endif
	#endif

	imu.initialize();

	delay(400);
	flash_on(0,status_bright,0,1);
}


//power
//0-254 - backwards; 255 - stop; 256-510 - forward

void sliderPWR() {
    if(authorisation==0)
		speed=255;

    if(speed>=255) {
    	int temp_speed=(speed-255)+(steer_angle-90)*(speed-255)/360;
    	if(temp_speed>255) temp_speed=255;

		if(powerlock==1) {
			temp_speed*=lspeed;
			temp_speed/=255;
		}
		digitalWrite(in1,HIGH);
		digitalWrite(in2,LOW);
		analogWrite(ena,temp_speed);

		//PCmL();

		temp_speed=(speed-255)-(steer_angle-90)*(speed-255)/360;
		if(temp_speed>255) temp_speed=255;
		if(powerlock==1) {
			temp_speed*=lspeed;
			temp_speed/=255;
		}
		digitalWrite(in3, HIGH);
		digitalWrite(in4, LOW);
		analogWrite(enb, temp_speed);

		//PCmR();
    }

	else {
		int temp_speed=(-speed+255)+(steer_angle-90)*(-speed+255)/360;
		if(temp_speed>255) temp_speed=255;
		if(powerlock==1) {
			temp_speed*=lspeed;
			temp_speed/=255;
		}
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
		analogWrite(ena, temp_speed);

		//PCmL();

		temp_speed=(-speed+255)-(steer_angle-90)*(-speed+255)/360;
		if (temp_speed>255) temp_speed=255;
		if(powerlock==1) {
			temp_speed*=lspeed;
			temp_speed/=255;
		}
		digitalWrite(in3, LOW);
		digitalWrite(in4, HIGH);
		analogWrite(enb, temp_speed);

		//PCmR();

	}
}


//range 0-180
void sliderSTR() {
	serwomechanizm.write(steer_angle);
	if(acs==1) {
		pan_pos=pan_zero+(steer_angle-90)/4;
		servopan.write(pan_pos);
		servotilt.write(180-tilt_zero);
	}
}


//power
//0-254 - backwards; 255 - stop; 256-510 - forward

void autoPWR(int speed_l, int speed_r) {
    if( authorisation == 0 )
		return;

    if( speed_l >= 255 ) {
    	int temp_speed = speed_l - 255;
    	if( temp_speed > 255 ) temp_speed = 255;

		digitalWrite(in1,HIGH);
		digitalWrite(in2,LOW);
		analogWrite(ena,temp_speed);
	}
	else {
		int temp_speed = -speed_l + 255;
		if(temp_speed>255) temp_speed=255;
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
		analogWrite(ena, temp_speed);
	}

	if( speed_r >= 255 ) {
    	int temp_speed = speed_r - 255;
    	if( temp_speed > 255 ) temp_speed = 255;

		digitalWrite(in3,HIGH);
		digitalWrite(in4,LOW);
		analogWrite(enb,temp_speed);
	}
	else {
		int temp_speed = -speed_r + 255;
		if(temp_speed>255) temp_speed=255;
		digitalWrite(in3, LOW);
		digitalWrite(in4, HIGH);
		analogWrite(enb, temp_speed);
	}
}


void CAM() {
	servopan.write(pan_pos);
	servotilt.write(180-tilt_pos);
}


  /*void speedometer()
    {
      if(powerlock==1)
      {
      tempspd=speed-255;
      tempspd*=lspeed;
      tempspd/=255;
      tempspd+=255;
      Serial1.print("*V"+String(tempspd)+"*");
      }
      else
      Serial1.print("*V"+String(speed)+"*");
    }*/


void tankL() {
    steer_angle=90;
    serwomechanizm.write(steer_angle);

    speed=255;
    //speedometer();

	if(powerlock==1)
		tankspeed=lspeed;
	else
		tankspeed=255;
	if(authorisation==0)
    	tankspeed=0;

    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(ena,tankspeed);

    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enb,tankspeed);
}


void tankR() {
	steer_angle=90;
	serwomechanizm.write(steer_angle);

	speed=255;
	//speedometer();

	if(powerlock==1)
		tankspeed=lspeed;
	else
		tankspeed=255;
	if(authorisation==0)
		tankspeed=0;

	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	analogWrite(ena,tankspeed);

	digitalWrite(in3,LOW);
	digitalWrite(in4,HIGH);
	analogWrite(enb,tankspeed);
}


void stop_rover() {
    speed=255;
    sliderPWR();
    //speedometer();
    digitalWrite(lsr,LOW);
    is_tank=0;
}


void reset_rover() {
    speed =255;
    steer_angle =90;
    pan_pos=pan_zero;
    tilt_pos=tilt_zero;
    sliderSTR();
    sliderPWR();
    CAM();
    //speedometer();
    digitalWrite(lsr,LOW);
    lighter_status=1;
    lighter();
    is_tank=0;
    authorisation = 0;
}


/*
//sensors
  void dht11_f()
  {
     int humid=dht11.getHumidity();
     int temp=dht11.getTemperature();
  if(dht11.getStatusString()=="OK")
  {
     Serial1.print("*T"+String(temp)+"*");
     Serial1.print("*H"+String(humid)+"*");
  }
  else
  {
    Serial1.print("*TERROR*");
     Serial1.print("*HERROR*");
  }
  }

  void mq_2()
  {
    mq2_map2=mq2_map;
    mq2_analog=analogRead(A0);
    mq2_map=map(mq2_analog,0,1023,0,10000);

    if(mq2_map2>2000&&mq2_map<2000)
    {
    Serial1.print("*SLOW*");
    Serial1.print("*YR0G0B0*");
    }
    if(mq2_map2<2000&&mq2_map>2000)
    Serial1.print("*YR0G255B0*");

    if(mq2_map>2000)
    {
    Serial1.print("*S"+String(mq2_map)+"*");
    Serial1.print("X");
    }
  }*/


void amp() {
    amp_analog=analogRead(current);
    amp_map=map(amp_analog,0,1023,0,4000);
    if(amp_map>1500&&amp_map<2500) {
		amp_tab[amp_i%qtest]=amp_map-1500;
		amp_i++;
    }

    //Serial1.print("*a"+String(amp_map-1500-500+ampcalibrate)+"*");
}


void ampwrite() {
    amp_res=0;
    for(int i=0; i<qtest; i++)
    	amp_res+=amp_tab[i];
    amp_res/=qtest;
    if(amp_res>=500&&amp_res<1000)
    	Serial1.print("*a"+String(amp_res-500+ampcalibrate)+"*");
}


void volting() {
    volt_analog=analogRead(voltage);
    volt=map(volt_analog,0,1023,0,2500);
    volt_res=volt;
    volt_res/=100;
    if(volt>=0&&volt<=2500) {
      	Serial1.print("*v"+String(volt_res, 1)+"*");
    }
    else
    	Serial1.print(F("*vERROR*"));
    if(volt<volt_zero||volt>2500) {
      	Serial1.print(F("*yR255G0B0*"));
    }
    else
   		Serial1.print(F("*yR0G0B0*"));
}


void lasergun() {
    laser_status=!laser_status;
    if(laser_status==1)
    digitalWrite(lsr,HIGH);
    else
    digitalWrite(lsr,LOW);
}


void lighter() {
    lighter_status=!lighter_status;
    if(lighter_status==1) {
		uint32_t head_delay_btw=13;
		uint32_t head_delay_grow=31;
		uint32_t a;
		uint32_t b;
		uint32_t b2;

		for(uint32_t i=0; i<=2*head_delay_btw+head_delay_grow; i++) {
			if (i<head_delay_grow)
				a=head_bright_2+i*(255-head_bright_2)/head_delay_grow;
			else
				a=255;

			head_led.setPixelColor(2, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.setPixelColor(3, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.show();

			if (i<head_delay_btw)
				a=head_bright_2;
			else if(i<head_delay_grow+head_delay_btw)
				a=head_bright_2+(i-head_delay_btw)*(255-head_bright_2)/head_delay_grow;
			else
				a=255;

			head_led.setPixelColor(1, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.setPixelColor(4, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.show();

			if (i<2*head_delay_btw)
				a=head_bright_2;
			else if(i<head_delay_grow+2*head_delay_btw)
				a=head_bright_2+(i-2*head_delay_btw)*(255-head_bright_2)/head_delay_grow;
			else
				a=255;

			head_led.setPixelColor(0, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.setPixelColor(5, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.show();

			if (i<2*head_delay_btw) {
				b=i*status_bright/2/head_delay_btw;
				b2=status_bright;
			}
			else if(i<head_delay_grow+2*head_delay_btw) {
				b=status_bright+(i-2*head_delay_btw)*(255-status_bright)/head_delay_grow;
				b2=b;
			}
			else {
				b=255;
				b2=b;
			}

			stat_led.setPixelColor(0, pgm_read_byte(&gamma8[b]), pgm_read_byte(&gamma8[b2]), pgm_read_byte(&gamma8[b]));
			stat_led.show();

			delay(8);
      	}

      //stat_led.setPixelColor(0,stat_led.Color(255,255,255));
      //stat_led.show();
    }
    else {
		uint32_t head_delay_btw=13;
		uint32_t head_delay_grow=31;
		uint32_t a;
		uint32_t b;
		uint32_t b2;

		for(uint32_t i=2*head_delay_btw+head_delay_grow; i>0; i--) {
			if (i<head_delay_grow)
				a=head_bright_2+i*(255-head_bright_2)/head_delay_grow;
			else
				a=255;

			head_led.setPixelColor(2, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.setPixelColor(3, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.show();

			if (i>head_delay_grow+head_delay_btw)
				a=255;
			else if(i>head_delay_btw)
				a=head_bright_2+(i-head_delay_btw)*(255-head_bright_2)/head_delay_grow;
			else
				a=head_bright_2;

			head_led.setPixelColor(1, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.setPixelColor(4, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.show();

			if (i>head_delay_grow+2*head_delay_btw)
				a=255;
			else if(i>2*head_delay_btw)
				a=head_bright_2+(i-2*head_delay_btw)*(255-head_bright_2)/head_delay_grow;
			else
				a=head_bright_2;

			head_led.setPixelColor(0, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.setPixelColor(5, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
			head_led.show();

			if (i>head_delay_grow+2*head_delay_btw) {
				b=255;
				b2=b;
			}
			else if(i>2*head_delay_btw) {
				b=status_bright+(i-2*head_delay_btw)*(255-status_bright)/head_delay_grow;
				b2=b;
			}
			else {
				b=i*status_bright/2/head_delay_btw;
				b2=status_bright;
			}

			stat_led.setPixelColor(0, pgm_read_byte(&gamma8[b]), pgm_read_byte(&gamma8[b2]), pgm_read_byte(&gamma8[b]));
			stat_led.show();

			delay(8);
      	}

		//stat_led.setPixelColor(0,stat_led.Color(0,pgm_read_byte(&gamma8[status_bright]),0));
		//stat_led.show();
    }
}


void flash_on(int status_r, int status_g, int status_b, int status_delay) {
	for (int i=0; i<=255;i++) {
		stat_led.setPixelColor(0,stat_led.Color(pgm_read_byte(&gamma8[i*status_r/255]),pgm_read_byte(&gamma8[i*status_g/255]),pgm_read_byte(&gamma8[i*status_b/255])));
		stat_led.show();
		delay(status_delay);
	}
}


void flash_off(int status_r, int status_g, int status_b, int status_delay) {
	for (int i=255; i>=0;i--) {
		stat_led.setPixelColor(0,stat_led.Color(pgm_read_byte(&gamma8[i*status_r/255]),pgm_read_byte(&gamma8[i*status_g/255]),pgm_read_byte(&gamma8[i*status_b/255])));
		stat_led.show();
		delay(status_delay);
		stat_led.clear();
	}
}


uint32_t lights_ini() {
	//status blink
	for(int i=0; i<2;i++) {
       stat_led.setPixelColor(0, 0, pgm_read_byte(&gamma8[status_bright]), 0);
       stat_led.show();
       delay(180);
       stat_led.setPixelColor(0, 0, 0, 0);
       stat_led.show();
       delay(250);
	}

	delay(600);

	//turn signals
	for(int k=0;k<2;k++) {
		for(int i=3; i>=0;i--) {
			head_led.clear();
			for(int j=0; j<i; j++) {
				head_led.setPixelColor(j, pgm_read_byte(&gamma8[head_bright]), pgm_read_byte(&gamma8[head_bright/2]), 0);
				head_led.setPixelColor(5-j, pgm_read_byte(&gamma8[head_bright]), pgm_read_byte(&gamma8[head_bright/2]), 0);
			}
			head_led.show();
			delay(55);
       }
       delay(250);
	}

	delay(200);

	//heads

	uint32_t head_delay_btw=220;
	uint32_t head_delay_grow=500;
	uint32_t head_delay_drop=170;
	uint32_t head_bright_2=head_bright-((head_delay_drop)*head_bright/head_delay_grow);
	uint32_t a;

	for(int i=0; i<=2*head_delay_btw+head_delay_grow+head_delay_drop; i++) {
		if (i<head_delay_grow)
			a=i*head_bright/head_delay_grow;
		else if (i<head_delay_grow+head_delay_drop)
			a=head_bright-((i-head_delay_grow)*head_bright/head_delay_grow);
		else
			a=head_bright_2;

		head_led.setPixelColor(0, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
		head_led.setPixelColor(5, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
		head_led.show();

		if (i<head_delay_btw)
			a=0;
		else if(i<head_delay_grow+head_delay_btw)
			a=(i-head_delay_btw)*head_bright/head_delay_grow;
		else if (i<head_delay_grow+head_delay_drop+head_delay_btw)
			a=head_bright-((i-head_delay_grow-head_delay_btw)*head_bright/head_delay_grow);
		else
			a=head_bright_2;

		head_led.setPixelColor(1, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
		head_led.setPixelColor(4, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
		head_led.show();

		if (i<2*head_delay_btw)
			a=0;
		else if(i<head_delay_grow+2*head_delay_btw)
			a=(i-2*head_delay_btw)*head_bright/head_delay_grow;
		else if (i<head_delay_grow+head_delay_drop+2*head_delay_btw)
			a=head_bright-((i-head_delay_grow-2*head_delay_btw)*head_bright/head_delay_grow);
		else
			a=head_bright_2;

		head_led.setPixelColor(2, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
		head_led.setPixelColor(3, pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]), pgm_read_byte(&gamma8[a]));
		head_led.show();

		delay(1);
	}
	return head_bright_2;
}


void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize)
{
	delay(100);
	uint8_t byteread, index;
	for (index = 0; index < arraysize; index++)	{
		byteread = pgm_read_byte_near(Progmem_ptr++);
		gpsPort.write(byteread);
	}
	delay(100);
}




void loop()
{
	/*  BTstate= digitalRead(bts);
	if (BTstate==0)
	{
		stop_rover();
		flash_off(0,status_bright,0,2); // to trwa i może kiedys cos psuc!
		flash_on(0,status_bright,0,2); // to trwa i może kiedys cos psuc!
	}*/

	//AUTONOMOUS MODE

	if(digitalRead(modeSW)==LOW)
	{
	//initiation aka setup

		if(mode==0)
		{
			mode=1;

			//leds
			stat_led.setPixelColor(0,stat_led.Color(0,0,pgm_read_byte(&gamma8[status_bright])));
			stat_led.show();

			//gps
			Serial.println(F("GPS_ini"));
			while (gps.available( gpsPort )) fix = gps.read();
			while (!fix.valid.location&&!skip) {
				if(Serial.available()) {
					rx_cmd=Serial.read();
					if(rx_cmd=='s') skip=true;
				}
				while (gps.available( gpsPort )){
					fix = gps.read();
					// Serial.println(fix.satellites);
				}
			}
			if(skip==0) Serial.println(F("ok"));
			else Serial.println(F("GPS_skip"));
			skip=0;

			authorisation=0;

			//compass
			Serial.println(F("IMU_ini"));
			while (!imu.testConnection());
			// while (!imu.testConnection()&&!skip) {
			// 	if(Serial.available())	{
			// 		rx_cmd=Serial.read();
			// 		if(rx_cmd=='s') skip=true;
			// 	}
			// }
			// if(skip==0) Serial.println(F("ok"));
			// else Serial.println(F("COS_skip"));
			// skip=0;
		}

	//base aka loop


		// GPS reading
		if(Serial.availableForWrite()){
			while (gps.available( gpsPort )){
				fix = gps.read();
				if(fix.valid.location) {
					Serial.println("G");
					Serial.println(fix.latitude(), 7);
					Serial.println(fix.longitude(), 7);
					Serial.println(fix.altitude(), 2);
					Serial.println(fix.lat_err(), 1);
					Serial.println(fix.lon_err(), 1);
					Serial.println(fix.alt_err(), 1);
				}
			}
		}

		//setting coordinates and safety, from radio

		if(Serial1.available()) {
			rx_cmd=Serial1.read();
			switch(rx_cmd) {
				//safety system
				case 'q':
					time_m_upd=millis();
					if(authorisation==0) {
						authorisation=1;
						Serial1.println(F("COM_ok"));
					}
					break;
			}
		}


		//PC control
		if(Serial.available()) {
			rx_cmd=Serial.read();
			if(rx_cmd=='D'){
				int speed_l = Serial.parseInt();
				int speed_r = Serial.parseInt();
				steer_angle = Serial.parseInt();
				autoPWR(speed_l, speed_r);
				sliderSTR();
				time_m_upd2=millis();
			}
		}


		// IMU
		if(Serial.availableForWrite()){
			getImu_Data();
			Serial.println("A");
			Serial.println(Axyz[0], 7);
			Serial.println(Axyz[1], 7);
			Serial.println(Axyz[2], 7);
			Serial.println(Gxyz[0], 7);
			Serial.println(Gxyz[1], 7);
			Serial.println(Gxyz[2], 7);
			Serial.println(Mxyz[0], 7);
			Serial.println(Mxyz[1], 7);
			Serial.println(Mxyz[2], 7);
		}


		//safety protocol
		time_m=millis();
		if(time_m-time_m_upd>3000) {
			if(authorisation==1) {
				authorisation=0;
				Serial1.println(F("COM_err"));
			}
		}
		if(time_m-time_m_upd2>500) {
			stop_rover();
		}


	}

	//REMOTE CONTROL MODE

	else {
	//initiation aka setup

		if(mode==1)	{
			mode=0;
			stat_led.setPixelColor(0,stat_led.Color(0,pgm_read_byte(&gamma8[status_bright]),0));
			stat_led.show();
			authorisation=0;
		}

	//base aka loop

		if (Serial1.available() > 0) {
			rx_cmd = Serial1.read();
			switch(rx_cmd)
			{
				case 'Q':
					authorisation=true;
					break;

				case 'A':{
					speed=Serial1.parseInt() - 200;
					char _=Serial1.read();
					steer_angle=Serial1.parseInt() - 200;
					_ = Serial1.read();
					if(speed>=0&&steer_angle>=0)
					{
						//Serial.println("D: "+String(speed)+", S: "+String(steer_angle));
						if(is_tank==0)
						{
						sliderSTR();
						sliderPWR();
						}
						//speedometer();
						time_m_upd3=millis();
					}
					break;
				}

				//	case 'C':
				//		dht11_f();
				//		break;

				case 'D':
					stop_rover();
					break;

				case 'S':{
					char _ = Serial1.read();
					pan_pos=Serial1.parseInt() - 200;
					_ = Serial1.read();
					tilt_pos=Serial1.parseInt() - 200;
					_ = Serial1.read();
					if(pan_pos>=0&&tilt_pos>=0&&pan_pos<=180&&tilt_pos<=180)
						CAM();
					break;
				}
				case 'F':
					acs=1;
					break;

				case 'G':
					acs=0;
					break;

				case 'c':
					pan_pos=pan_zero;
					tilt_pos=tilt_zero;
					CAM();
					break;

				case 'L':
					tankL();
					is_tank=1;
					break;

				case 'l':
					stop_rover();
					is_tank=0;
					break;

				case 'R':
					tankR();
					is_tank=1;
					break;

				case 'r':
					stop_rover();
					is_tank=0;
					break;

				case 'H':
					reset_rover();
					break;

				case 'J':
					powerlock=0;
					break;

				case 'j':
					powerlock=1;
					break;

				case 'U':
					lasergun();
					break;

				case 'W':        //OBECNIE LATARKA MOŻE POWODOWAĆ PRZERWĘ W POŁĄCZENIU => BLOKADA !!!!!!!!!!!!!!!!!!!!!!!!
					lighter();
					break;

				case 'v':
					volting();
					break;

				case 'a':
					ampwrite();
					break;
			}
			//PC();
		}

		time_m=millis();


		if((time_m-time_m_upd2)>100)
		{
			amp();
			time_m_upd2=time_m;
		}

		//co 2 sek
		if((time_m-time_m_upd3)>2500)
		{
			stop_rover();
			authorisation = 0;
			time_m_upd3=time_m;
		}

		//mq_2();
		//fs();
	}
}

//compass functions

// void getTiltHeading(void) {
// 	roll = asin(-Axyz[0])-roll_center;
// 	pitch = asin(Axyz[1]/cos(roll))-pitch_center;

// 	float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
// 	float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
// 	float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
// 	tiltheading = -180 * atan2(yh, xh)/PI;
// 	if(yh>0)    tiltheading +=360;
// }



/*void Mxyz_init_calibrated () {

  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  while(!Serial.find("ready"));
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  get_calibration_Data ();

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}


void get_calibration_Data () {
    for (int i=0; i<sample_num_mdate;i++)
      {
      get_one_sample_date_mxyz();

      Serial.print(mx_sample[2]);
      Serial.print(" ");
      Serial.print(my_sample[2]);                            //you can see the sample data here .
      Serial.print(" ");
      Serial.println(mz_sample[2]);




      if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];
      if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value
      if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];

      if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
      if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
      if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];

      }

      mx_max = mx_sample[1];
      my_max = my_sample[1];
      mz_max = mz_sample[1];

      mx_min = mx_sample[0];
      my_min = my_sample[0];
      mz_min = mz_sample[0];



      mx_centre = (mx_max + mx_min)/2;
      my_centre = (my_max + my_min)/2;
      mz_centre = (mz_max + mz_min)/2;

}*/


void getImu_Data(void) {
	imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	Axyz[0] = (double) ax / 16384;//16384  LSB/g
	Axyz[1] = (double) ay / 16384;
	Axyz[2] = (double) az / 16384;
	Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
	Gxyz[1] = (double) gy * 250 / 32768;
	Gxyz[2] = (double) gz * 250 / 32768;
	Mxyz[0] = (double) mx * 4800 / 8192 - mx_centre;
	Mxyz[1] = (double) my * 4800 / 8192 - my_centre;
	Mxyz[2] = (double) mz * 4800 / 8192 - mz_centre;
}


const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };
