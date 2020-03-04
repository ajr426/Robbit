
//Arduino Pro Mini (remote controller)
//Joystick data is read into the arduino and sent to the robot in order to control it

#include <LiquidCrystal.h>

#include <XBee.h>

/*
This example is for Series 2 XBee
 Sends a ZB TX request with the value of analogRead(pin5) and checks the status response for success
*/

// create the XBee object
XBee xbee = XBee();

uint8_t payload[] = { 0, 0 };

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40b40397);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

//LCD pins
//RS pin = pin 12
//Enable pin = pin 11
//D4 = pin 5
//D5 = pin 4
//D6 = pin 3
//D7 = pin 2
//R/W = gnd
//VSS = gnd
//VCC = 5V
//220 ohm resistor b/t 5V and LED+
//pot wiper to V0
//Xbee DOUT = RXI
//Xbee DIN = TX0
//Joystick Vrx = A0
//Joystick Vry = A1
//Power supply Vcc = VCC
//Power supply ground = GND

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//joystick pins
int x1pin = A0;           //joystick x pin connected to arduino
int y1pin = A1;           //joystick y pin connected to arduino

bool send_data = true;   //determines if joystick data should be sent to robot
int diffx;               //difference between current x position and previous x position
int diffy;               //difference between current y position and previous y position

struct remote_data       //data sent from remote to robot, joystick positions
{
  int jx;          //joystick x position
  int jy;          //joystick y position
};

struct robot_data  //data sent from robot to remote
{
  int intensity;  //metal detector instensity detected
  int battery;    //robot battery level
};

remote_data output_data;    //data output to xbee module
remote_data prev_data;      //previous positon data used to see if data should be transmitted
remote_data raw_data;       //raw data read in from joystick

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);  //intitializes communication with xbee pro s2b
  xbee.setSerial(Serial); //sets Serial to xbee library


  //LCD screen setup
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Starting up!");

  //nitializes joystick positions
  output_data.jx = 0;
  output_data.jy = 0;

  pinMode(LED_BUILTIN, OUTPUT);   //LED used to indicate when data is sent from joystick
}

void loop()
{

  //read in data from joystick (0 to 1023 range)
  raw_data.jx = analogRead(x1pin);
  raw_data.jy = analogRead(y1pin);

  //change 0 to 1023 range to 0 to 254 range
  //xbee sends byte (0 to 255 range) wirelessly
  //we reserve 255 to declare the start of a data packet
  payload[0] = map(raw_data.jx, 0, 1023, 0, 254);
  payload[1] = map(raw_data.jy, 0, 1023, 0, 254);

  //determines if joystick position has changed
  //Joystick position is only sent to robot if it has changed
  diffx = abs(raw_data.jx - prev_data.jx);
  diffy = abs(raw_data.jy - prev_data.jy);

  //If the difference is bigger than 10, then send data to robot
  if(diffx > 10 || diffy > 10)
  {
    send_data = true;
  }
  else
  //this means that the joystick position has not changed and no new data should be sent to robot
  {
    send_data = false;
    digitalWrite(LED_BUILTIN,LOW);   //used for debugging purposes
    lcd.clear();
    lcd.print("No joystick change");
  }

  //where data is actually sent if joystick position has changed
  //data is written to xbee module through serial connection
  //xbee then transmits that data wirelessly
  if (send_data)
  {
    xbee.send(zbTx);
    digitalWrite(LED_BUILTIN,HIGH);  //indicates if joystick has been moved
    lcd.clear();
    lcd.print("Sent x:");
    lcd.print(payload[0]);
    lcd.setCursor(0,1);
    lcd.print("Sent y:");
    lcd.print(payload[1]);
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

        } else {
          // the remote XBee did not receive our packet. is it powered on?

        }
      }
    } else if (xbee.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");
      //nss.println(xbee.getResponse().getErrorCode());
    } else {
      // local XBee did not provide a timely TX Status Response -- should not happen

    }

  }

  delay(20);          //makes program run better

}
