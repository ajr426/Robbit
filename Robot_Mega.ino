//Arduino Mega (on robot)
//Processes remote control data, then outputs to motor driver in order to control robot
//Interfaces with metal detector to determine is metal is detected

//USB port plugged into computer
//Xbee DOUT plugged into arduino RX1 (19)
//Xbee DIN plugged into arduino TX1 (18)

#include "arduinoFFT.h"  //include FFT library

//FFT variables
//from https://www.norwegiancreations.com/2019/03/arduino-fft-pt-2-improving-the-hardware-for-real-time-analysis/
#define SAMPLES 512               //Must be a power of 2
#define SAMPLING_FREQUENCY 6400  //Hz
#define REFRESH_RATE 10           //Hz
#define ARDUINO_IDE_PLOTTER_SIZE 500
arduinoFFT FFT = arduinoFFT();

unsigned long sampling_period_us;
unsigned long useconds_sampling;

unsigned long refresh_period_us;
unsigned long useconds_refresh;

double vReal[SAMPLES];
double vImag[SAMPLES];
double peak;        //most dominant frequency in metal detector signal

uint8_t analogpin = A0;         //metal detector input pin

//Xbee communication setup
int incoming;
int rx_buffer[3] = {0,0,0};     //incoming data from remote controller
int rx_buffer_size = 3;
int buffer_counter = 0;
bool packet_good = true;

//Used for flow control to/from xbee UART
int rts_pin = 2;
int cts_pin = 3;

struct remote_data       //data sent from remote to robot, joystick positions
{
  int packet_start;  //number used to indicate start of packet
  int jx;           //joystick x position
  int jy;           //joystick y position
};

remote_data joy_data;

void setup() {
  // initialize both serial ports
  //Serial (Serial0) used to communicate with computer
  //Serial1 used to communicate with xbee pro s2b
  Serial.begin(115200);
  Serial1.begin(9600);

  //pin used for rts flow control on xbee pro s2b
  //RTS flow control allows the host to signal the module to not send data in the serial transmit buffer out the UART (serial)
  //CTS flow control provides an indication to the host to stop sending serial data to the module.
  pinMode(rts_pin,OUTPUT);
  pinMode(cts_pin,INPUT);

  //on board LED used to indicate when joystick data is recieved
  pinMode(LED_BUILTIN, OUTPUT);

  //FFT setup
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  refresh_period_us = round(1000000*(1.0/REFRESH_RATE));

  pinMode(analogpin, INPUT);

}

void loop()
{

  if (Serial1.available())
  {
    while (buffer_counter < rx_buffer_size)
    {
      digitalWrite(LED_BUILTIN,HIGH);
      rx_buffer[buffer_counter] = Serial.read();
      buffer_counter++;
    }
  }

  //checks if packet is valid then stores joystick positions
  validate_packet();
  delay(50);
  digitalWrite(LED_BUILTIN,LOW);

  //controls motors based on remote controller input
  motor_control();

  //takes fft of metal detector input
  fft();

}

void validate_packet(){
  if(rx_buffer[0] != 255)
  {
    packet_good = false;
  }
  else if(rx_buffer[1] == 255 || rx_buffer[2] == 255)
  {
    packet_good = false;
  }
  else
  {
    packet_good = true;
  }

  if(packet_good)
  {
    joy_data.packet_start = rx_buffer[0];
    joy_data.jx = rx_buffer[1];
    joy_data.jy = rx_buffer[2];
  }

}

void motor_control(){

}

void fft(){
  useconds_refresh = micros();

  /*SAMPLING*/
  for(int i=0; i<SAMPLES; i++)
  {
    useconds_sampling = micros();

    vReal[i] = analogRead(analogpin);
    vImag[i] = 0;

    while(micros() < (useconds_sampling + sampling_period_us)){
      //wait...
    }
  }

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);  //determines strongest frequency in signal

  //prints out peak frequency
  Serial.print("Peak frequency:");
  Serial.println(peak);

  //not using these print results, they plot it to the arduino plotter
  //PRINT RESULTS
  /*
  for(int i=0; i<(SAMPLES/2); i++){
    Serial.println(vReal[i], 1);
  }
  for(int i=0; i<(ARDUINO_IDE_PLOTTER_SIZE - (SAMPLES/2)); i++){
    Serial.println(0);
  }

  //dont think i need this, it really is used for when fft is only thing in loop
  //can be replaced by timer?
  while(micros() < (useconds_refresh + refresh_period_us)){
    //wait...
  }
  */
}
