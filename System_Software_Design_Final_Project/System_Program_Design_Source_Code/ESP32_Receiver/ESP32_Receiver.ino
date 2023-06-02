#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <analogWrite.h>

const int servo_up_down = 32;
const int servo_left_right = 25;
const int pwm_controller_in1 = 27;
const int pwm_controller_in2 = 12;

 void ServoRotate(int angle , int servopin) //Define Servo PWM Function
 {
    int pulsewidth=(angle*11)+500;  //Convert the angle into a pulse width value of 500-2480, every 1 degree of rotation, the corresponding high level is 11us more
    digitalWrite(servopin,HIGH);    //Set the servo interface level to high
    delayMicroseconds(pulsewidth);  //Delay pulse width value in microseconds
    digitalWrite(servopin,LOW);     //Set the servo interface level to low
    delayMicroseconds(20000-pulsewidth);   
} 
// message struct received
typedef struct message {
   int servo_up_down;
   int servo_left_right;
   int motor_pwm;
   int motor_start;
} message;

uint8_t key[] =  {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};
message myMessage;

void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) {
   //Serial.println("Message received.");
   
  memcpy(&myMessage, incomingData, sizeof(myMessage)); //transform the incomingData into our message structure

  Serial.println(myMessage.servo_up_down);
  ServoRotate(myMessage.servo_up_down,servo_up_down); //control up,down servo
  delay(10);

  Serial.println(myMessage.servo_left_right);
  ServoRotate(myMessage.servo_left_right,servo_left_right); //control left,right servo
  delay(10);
  
  //motor control MX1508
  if(myMessage.motor_start == 0){       //shut down motor
    analogWrite(pwm_controller_in1, 255);
    digitalWrite(pwm_controller_in2, HIGH);
  }
  else{
    Serial.println(myMessage.motor_pwm);
    analogWrite(pwm_controller_in1, myMessage.motor_pwm);  //use joystick control MX1508 output voltage , so we can control the motor rotating speed
    digitalWrite(pwm_controller_in2, HIGH);
  }
  
  
}
void setup() {
  Serial.begin(115200);

  pinMode(pwm_controller_in2, OUTPUT); //IN2
  pinMode(pwm_controller_in1, OUTPUT); //IN1

  pinMode(servo_up_down,OUTPUT);
  pinMode(servo_left_right,OUTPUT);

  // Wifi STA Mode
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("\nESP-Now Receiver");
  
  // Initializing the ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  
  //esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceiver);
}
void loop() {
}
