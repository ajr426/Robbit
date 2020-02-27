//Arduino Mega (on robot)
//Processes remote control data, then outputs to motor driver in order to control robot
//Interfaces with metal detector to determine is metal is detected

int incoming;
int rx_buffer[3] = {0,0,0};     //incoming data from remote controller
int rx_buffer_size = 3;
int buffer_counter = 0;
bool packet_good = true;

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
  Serial.begin(9600);
  Serial1.begin(9600);

  //on board LED used to indicate when joystick data is recieved
  pinMode(LED_BUILTIN, OUTPUT);
  
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
 
  delay(50);
  digitalWrite(LED_BUILTIN,LOW);

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
  
