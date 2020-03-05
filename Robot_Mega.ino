//Arduino Mega (on robot)
//Processes remote control data, then outputs to motor driver in order to control robot
//Interfaces with metal detector to determine is metal is detected

//USB port plugged into computer
//Xbee DOUT plugged into arduino RX1 (19)
//Xbee DIN plugged into arduino TX1 (18)

#include "arduinoFFT.h"  //include FFT library
#include <Xbee.h>        //include xbee comms library

//FFT variables
//from https://www.norwegiancreations.com/2019/03/arduino-fft-pt-2-improving-the-hardware-for-real-time-analysis/
#define SAMPLES 512                     //Must be a power of 2, number of samples used in fft calculation
#define SAMPLING_FREQUENCY 6400         //Hz, frequency to take analog input samples, must be less than 10000 due to arduino ADC
#define REFRESH_RATE 10                 //Hz, used for timing when fft is called
#define ARDUINO_IDE_PLOTTER_SIZE 500    //size for plotting results to arduino ide plotter
arduinoFFT FFT = arduinoFFT();          //create fft object

unsigned long sampling_period_us;       //time between fft samples
unsigned long useconds_sampling;        //micros() used to precisely time samples

//unsigned long refresh_period_us;        //time between fft function calls
//unsigned long useconds_refresh;         //micros() used to precisely time fft function

double vReal[SAMPLES];                  //real part of fft array
double vImag[SAMPLES];                  //imaginary part of fft array
double peak;        //most dominant frequency in metal detector signal

uint8_t analogpin = A0;         //metal detector input pin

//Xbee rx variables
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

//data sent from remote to robot, joystick positions
struct remote_data
{
  int jyl;           //left joystick y position
  int jyr;           //right joystick y position
  int rx_str;        //recieved transmission signal strength (RSSI)
};

remote_data joystick;

//xbee tx variables
uint8_t payload[] = { 0, 0 };
// SH + SL Address of receiving XBee
//The recieving xbee is the remote controller (arduino pro mini)
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40a78880);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

struct robot_data  //data sent from robot to remote
{
  int intensity;  //metal detector instensity detected
  int battery;    //robot battery level
};

robot_data tx_data;  //data transmitted to remote controller for user viewing

void setup() {

  // initialize both serial ports
  Serial.begin(9600);   //Serial (Serial0) used to communicate with computer
  Serial1.begin(9600);  //Serial1 used to communicate with xbee pro s2b

  xbee.begin(Serial1);  //assign Serial1 to xbee object use

  Serial.println("Starting up!");

  //FFT setup
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  refresh_period_us = round(1000000*(1.0/REFRESH_RATE));

  pinMode(analogpin, INPUT);   //setup pin for metal detector input

}

void loop()
{
  //receive data from remote controller
  rf_recieve();

  //controls motors based on remote controller input
  motor_control();

  //takes fft of metal detector input
  fft();

  //process metal detector and battery data before transmitting to user
  process_data();

  //send data to remote controller for user to view
  rf_transmit();

}

//receives data from remote controller
//assigns joystick positions to variables in joystick structure
void rf_recieve(){
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    // got something
    //Serial.println("Got something");

    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      // got a zb rx packet
      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx);
      //Serial.println("Got an rx packet!");

      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
          // the sender got an ACK
          //Serial.println("packet acknowledged");
          joystick.jyl = rx.getData()[0];
          joystick.jyr = rx.getData()[1];
          joystick.rx_str = rx.getRssi();
          Serial.println(joystick.jyl);
          Serial.println(joystick.jyr);
          Serial.println(joystick.rx_str);
      }
      else {
        //Serial.println("packet not acknowledged");
      }
    }
  }
  else if (xbee.getResponse().isError()) {
    //Serial.print("error code:");
    //Serial.println(xbee.getResponse().getErrorCode());
  }
}

//used to control drive motors based on joystick input
void motor_control(){

}

//fft of metal detector audio signal
//stores strongest frequency present in the signal
void fft(){
  //useconds_refresh = micros();          //starting time for fft call

  /*SAMPLING*/
  for(int i=0; i<SAMPLES; i++)
  {
    useconds_sampling = micros();        //overflows after 70 minutes

    vReal[i] = analogRead(analogpin);
    vImag[i] = 0;

    //wait for sampling_period_us amount of time for consistent samples at sampling frequency
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
  //Serial.print("Peak frequency:");
  //Serial.println(peak);

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

//processes metal detector and battery data
void process_data(){
  //will figure out some way to represent the peak frequency
  //right now take what I think is max freq for our fft and map it to 8 bits (0-255)
  tx_data.intensity = map(peak, 0, 3200, 0, 255);
}

//transmit data to remote controller for user viewing
void rf_transmit(){

  //assign values to payload
  payload[0] = tx_data.intensity;
  payload[1] = tx_data.battery;

  //send data to remote controller
  xbee.send(zbTx);

  // after sending a tx request, we expect a status response
  // wait up to half second for the status response
  if (xbee.readPacket(500)) {
    // got a response!

    // should be a znet tx status
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate

      }
      else {
        // the remote XBee did not receive our packet. is it powered on?
      }
    }
  }
  else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");
    //nss.println(xbee.getResponse().getErrorCode());
  }
  else {
    // local XBee did not provide a timely TX Status Response -- should not happen
  }
}
