
//Arduino Pro Mini (remote controller)
//Joystick data is read into the arduino and sent to the robot in order to control it

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
  output_data.jx = map(raw_data.jx, 0, 1023, 0, 254);
  output_data.jy = map(raw_data.jy, 0, 1023, 0, 254);

  //determines if joystick position has changed
  //Joystick position is only sent to robot if it has changed
  diffx = abs(output_data.jx - prev_data.jx);
  diffy = abs(output_data.jy - prev_data.jy);

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
  }


  //where data is actually sent if joystick position has changed
  //data is written to xbee module through serial connection
  //xbee then transmits that data wirelessly
  if (send_data)
  {
    Serial.write(255);
    delay(20);                     //delays used to make sure xbee module has time to process data
    Serial.write(output_data.jx);
    delay(20);
    Serial.write(output_data.jy);
    delay(20);
    prev_data.jx = output_data.jx;   //used to calculate whether joystick position has changed
    prev_data.jy = output_data.jy;
    digitalWrite(LED_BUILTIN,HIGH);  //indicates if joystick has been moved
    
    //Serial.println("Sent Data");   //for debugging with computer
  }

  delay(20);          //makes program run better

}
