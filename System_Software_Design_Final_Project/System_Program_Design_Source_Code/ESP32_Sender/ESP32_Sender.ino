#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

//define pin
const int buttonPin = 19;  // motor switch
const char x_axis = A5;
const char y_axis = A4;
const char x_axis_motor = A6;
const char y_axis_motor = A3;

int vrx_servo,vry_servo;     // joystick x-axis & y-axis
int x_last,y_last; 
/*
if we rapidly push our joystick to x-axis or y-axis, x-axis&y-axis will all become 3545 , to solve this problem , 
we record last time vrx_servo&vry_servo ,so if the problem occur ,we can use x_last & y_last to replace vrx_servo & vry_servo 
*/
int vrx_motor,vrx_motor,motorstart; // to control PWM controller MX1508

// ESP8266 Mac address (first peer)
uint8_t mac_peer1[] = {0x58,0xBF,0x25,0x34,0x31,0x34}; // ESP32 : 40:91:51:9A:1C:A0 ESP32_1 : 40:91:51:9B:27:34 Esp32U : 58:BF:25:34:31:34
//uint8_t broadcastAddress[] = {0xE8, 0xDB, 0x84, 0x9A, 0xE7, 0x0E};//sender:E8:DB:84:9A:E7:0E receiver:E8:DB:84:9B:45:60

esp_now_peer_info_t peer1;
int i = 0;
// message struct send to receiver
typedef struct message {
   int servo_up_down;
   int servo_left_right;
   int motor_pwm;
   int motor_start; 
};

struct message myMessage;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("ESP32 ESP-Now Broadcast");
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  memcpy(peer1.peer_addr, mac_peer1, 6);
  peer1.channel = 1;
  peer1.encrypt = 0;
  // Register the peer
  Serial.println("Registering a peer 1");
  if ( esp_now_add_peer(&peer1) == ESP_OK) {
    Serial.println("Peer 1 added");
  }  
  // define init value
  x_last = 1975;
  y_last = 1975;
}

void loop() {
  /* read joysstick value & Correction Value*/
  vrx_servo = analogRead(x_axis)-550;
  vry_servo = analogRead(y_axis)-550;
  /* avoid conflict */
  if((((analogRead(x_axis)-550 == 3545) && (analogRead(y_axis)-550 == 3545)) == true)){
    vrx_servo = x_last;
    vry_servo = y_last;
  }
  /* read joysstick value & Correction Value*/
  vrx_motor = analogRead(x_axis_motor)-550;
  vrx_motor = analogRead(y_axis_motor)-550;
  
  motorstart = digitalRead(buttonPin);
  
  char buf[100],motorbuf[100];

  /* Map the value to right angle & PWM Frquency*/
  myMessage.servo_up_down = constrain(map(vrx_servo, 0, 3545, 30, 150)-10,30,150);
  myMessage.servo_left_right = constrain(map(vry_servo, 0, 3545, 30, 150)-10,30,150);
  myMessage.motor_pwm = constrain(map(vrx_motor, 0, 3545, 0, 255),0,255);
  myMessage.motor_start = motorstart;
  /* show result on monitor */
  sprintf(buf, "VRx=%d, VRy=%d  AngleX=%d  AngleY=%d ", vrx_servo, vry_servo, myMessage.servo_up_down , myMessage.servo_left_right);
  Serial.println(buf);
  sprintf(motorbuf, "VRx_motor=%d, VRy_motor=%d ,motorstart=%d", vrx_motor, vry_motor, motorstart);
  Serial.println(motorbuf);
  //Serial.println("Send a new message");
  esp_now_send(NULL, (uint8_t *) &myMessage, sizeof(myMessage));

  x_last = vrx_servo;
  y_last = vry_servo;
  delay(100);
}
