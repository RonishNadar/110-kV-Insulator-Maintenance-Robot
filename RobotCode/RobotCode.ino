#include <PPMReader.h>
#include <Servo.h>

PPMReader ppm(interruptPin, channelAmount);
Servo myservo;

// Attach pin D2 Arduino to pin Echo of HC-SR04 & pin D3 Arduino to pin Trig of HC-SR04
#define echoPin 34
#define trigPin 32

// Defines variables
int pos = 0;        // Variable of servo position
int safety = 0;     // Safety variable used as precaution if controller fails

long duration;      // Variable for the duration of sound wave travel
float distance;     // Variable for the distance measurement
int left_cm = 4;    // Left ultrasonic limit
int right_cm = 20;  // Right ultrasonic limit
int center_cm = 12; // Center ultrasonic limit
float dist_cm;      // Variable for current distance

int automate = 0;     // Variable for line following automation
int arm_automate = 0; // Variable for arm cleaning automation

const int latchPin = 39;      // LED shift register latch
const int clockPin = 37;      // LED shift register clock
const int dataPin = 43;       // LED shift register data
const int numOfRegisters = 2; // LED no. of shift register 
byte* registerState;          // LED shift register state

byte interruptPin = 18; // Controller Interrupt
byte channelAmount = 8; // Controller channels

// Variables to control motor 1 linear actuator
const int AN1=8;
const int EN1=25;

// Variables to control motor 2 bldc
const int AN2=2;
const int EN2=45;

// Variables to control arm 1
const int AN5 = 9;
const int EN5 = 27;

// Variables to control arm 2
const int AN4 = 13;
const int EN4 = 30;

// Variables to control arm 3
const int AN3 = 12;
const int EN3 = 28;

// Variables to control lead screw
const int AN6 = 11;
const int EN6 = 26;
const int i_topLimitR = 31;         
const int i_bottomLimitL = 35;
const int i_topLimitL = 29;         
const int i_bottomLimitR = 33;

// Variables to control gimble
const int AN7 = 5;
const int EN7 = 22;
const int AN8 = 6;
const int EN8 = 23;

// Variables to control relays
const int bosch = 38;
const int water_pump = 40;
const int soap_pump = 42;
const int pneumatic = 47;
const int oil = 3;

// Variables for Gimble switch
const int gimble_left = 46;
const int gimble_right = 44;

// Variables for BLDC speed and direction
int bldc_speed = 0;
int bldc_dir = HIGH;

// Variables for gimble speed and direction
int gimble1_speed = 0;
int gimble2_speed = 0;
int gimble1_dir = HIGH;
int gimble2_dir = HIGH;

// Variables for linear actuator speed and direction
int linear_speed = 0;
int linear_dir = HIGH;

// Variables for SCARA speed and direction
int m1_speed = 0;
int m2_speed = 0;
int m3_speed = 0;
int m4_speed = 0;

int m1_dir = HIGH;
int m2_dir = HIGH;
int m3_dir = HIGH;
int m4_dir = HIGH;

int end_eff = 0;
int end_eff_timer =300;
int end_eff_dir = 0;

//////////////////////////////////////


//water check variables
const int i_floatSensorNC = 14;  const int i_floatSensorNO = 7;
int d_tankFull = 0;               int d_tankEmpty = 0;            int d_tankStatus = 0;

//Bosch Variables
const int o_boschRelay = 38;      int d_boschRelayStatus = 0;

//Water Pump Variables
const int o_pumpRelay =40;        int d_pumpRelayStatus = 0;

//Soap Pump Variable
const int o_soapPumpRelay = 42;    int d_soapPumpRelayStatus = 0;

//pneumatic Variables
const int o_pneumaticRelay = 47;  int d_pneumaticRelayStatus = 0;
 

//Ball screw and motor  variables
const int digitalMotor1 = 26;      const int analogMotor1 = 11;     const int ballScrewMotorSpeed = 170;

//Finite State Machine Variables
const int i_startProcess = 44;    int currentState = -1;/*-1*/           const int minusState = -1;/*-1*/ 

const int zeroState = 0;/*0*/           const int oneState = 1;    /*1*/       const int twoState = 2; /*2*/ 
const int threeState = 3;  /*3*/        const int fourState = 4; /*4*/         const int fiveState = 5;/*5*/

// Checks the water level of the tank returns either HIGH or LOW
int waterCheck(int tankFull, int tankEmpty){
  //declarations  
  int tankRelay;
  
  //Function body  
  if(tankEmpty == 0 && tankFull == 1){
    //Serial.println("Tank is Full");    
    tankRelay = 1;
  }       
  if(tankEmpty == 1 &&  tankFull == 0){
    //Serial.println("Tank is Empty");   
    tankRelay = 0;
  } 
  tankFull = tankEmpty = 0; 
  return tankRelay;
}

// Checks if the Bosch is either ON or OFF
int boschCheck(int bosch){
  //declerations
  int boschRelay;
  
  if (bosch == HIGH){
    boschRelay = 1;
    //Serial.println("Bosch is ON");      
  }
  else{ 
    boschRelay = 0;
    //Serial.println("Bosch is OFF");
  }
  bosch = 0;
  return boschRelay;
}

// Checks if the water pump is either ON or OFF
int pumpCheck(int pump){
  //declerations
  int pumpRelay;
  
  if (pump == HIGH){
    pumpRelay = 1;
    //Serial.println("Water Pump is ON");      
  }
  else{ 
    pumpRelay = 0;
    //Serial.println("Water Pump is OFF");
  }
  pump = 0;
  return pumpRelay;
}


// Checks if the Soap water pump is either ON or OFF
int soapPumpCheck(int soapPump){
  //declerations
  int soapPumpRelay;
  
  if (soapPump == HIGH){
    soapPumpRelay = 1;
    //Serial.println("Soap water pump is ON");      
  }
  else{ 
    soapPumpRelay = 0;
    //Serial.println("Soap water pump is OFF");
  }
  soapPump = 0;
  return soapPumpRelay;
}

// Checks if the water pneumatic is either ON or OFF
int pneumaticCheck(int pneumatic){
  //declaration
  int pneumaticRelay;

  if(pneumatic == HIGH){
    pneumaticRelay = 0;
    //Serial.println("Pneumatic valve is Open");
  }
  else{
    pneumaticRelay = 1;
    //Serial.println("pneumatic valve if Closed");
  }
  pneumatic = 0;
  return pneumaticRelay;
}


//////////////////////////////////////

// Steering distance calculation
float dist(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;

    return distance;
}

// Steering center calculation
void center(){

  // regWrite(2, HIGH);

  //Serial.println("Centering");
  // Serial.println(dist_cm);

  if(dist_cm < 12.50){
    //right
    linear_dir = HIGH;
    float dist_diff = 12.50 - dist_cm;
    if (dist_diff > 1){ 

      analogWrite(AN1,255);
    }else{
      
      analogWrite(AN1,105);
    }
    //analogWrite(AN1,255);
    digitalWrite(EN1,linear_dir);
    //delay(100);

  }else if(dist_cm > 12.50){
    //left
    linear_dir = LOW;
    float dist_diff = dist_cm - 12.50;
    if (dist_diff > 1){ 
      
      analogWrite(AN1,255);
    }else{
      
      analogWrite(AN1,105);
    }
    digitalWrite(EN1,linear_dir);
    //delay(100);

  }else{
    //stop
    analogWrite(AN1,0);
  }

  delay(100);

}

// Steering gimble rotate
void rotate(){
  int linear_dir = LOW;
  int linear_speed = 70;

  while(digitalRead(gimble_right) == 1){
    //Serial.print(digitalRead(gimble_right) );
    analogWrite(AN7,linear_speed);
    digitalWrite(EN7,linear_dir);
  }

  linear_dir = LOW;
  linear_speed = 0;
  analogWrite(AN7,linear_speed);
  digitalWrite(EN7,linear_dir);

  for (int i=0; i<3; i++){

    linear_speed = 70;
    linear_dir = HIGH;
    analogWrite(AN7,linear_speed);
    digitalWrite(EN7,linear_dir);
    Serial.print("X");
    Serial.print("X");
    Serial.print("X");
    delay(1000);

    linear_speed = 0;
    linear_dir = HIGH;
    analogWrite(AN7,linear_speed);
    digitalWrite(EN7,linear_dir);

    delay(1000);

  }

  while(digitalRead(gimble_right) == 1){
    //Serial.print(digitalRead(gimble_right) );
    analogWrite(AN7,linear_speed);
    digitalWrite(EN7,linear_dir);
  }

}


void setup() {
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  registerState = new byte[numOfRegisters];
  for (size_t i = 0; i < numOfRegisters; i++) {
    registerState[i] = 0;
  }
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Change PWM frequency
  TCCR3B = TCCR3B & B11111000 | B00000001; 
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  myservo.attach(3);

  // Motors bldc locomotion and linear actuator
  pinMode(AN1,OUTPUT);
  pinMode(AN2,OUTPUT);
  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);

  // Motors arm 1,2,3 and lead screw
  pinMode(AN3,OUTPUT);
  pinMode(EN3,OUTPUT);
  pinMode(AN4,OUTPUT);
  pinMode(EN4,OUTPUT);
  pinMode(AN5,OUTPUT);
  pinMode(EN5,OUTPUT);
  pinMode(AN6,OUTPUT);
  pinMode(EN6,OUTPUT);
  pinMode(AN7,OUTPUT);
  pinMode(EN7,OUTPUT);
  pinMode(AN8,OUTPUT);
  pinMode(EN8,OUTPUT);

  // Relays
  pinMode(bosch, OUTPUT);
  pinMode(water_pump, OUTPUT);
  pinMode(soap_pump, OUTPUT);
  pinMode(pneumatic, OUTPUT);
  pinMode(oil, OUTPUT);

  digitalWrite(bosch, LOW);
  digitalWrite(water_pump, LOW);
  digitalWrite(soap_pump, LOW);
  digitalWrite(pneumatic, HIGH);
  digitalWrite(oil, LOW);
  myservo.write(0);

  //Switches
  pinMode(gimble_right, INPUT);
  pinMode(gimble_left, INPUT);

  pinMode(i_topLimitR, INPUT);
  pinMode(i_topLimitL, INPUT);
  pinMode(i_bottomLimitL, INPUT);
  pinMode(i_bottomLimitR, INPUT);

  pinMode(49,OUTPUT);
  pinMode(51,OUTPUT);
  pinMode(53,OUTPUT);
  pinMode(10, OUTPUT);

  for(int i = 0;i<16;i++){
    regWrite(i,LOW);
  }

  delay(1500);
}

void loop() {
  int led_arr[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  int in1 = ppm.latestValidChannelValue(1, 0);
  int in2 = ppm.latestValidChannelValue(2, 0);
  int in3 = ppm.latestValidChannelValue(3, 0);
  int in4 = ppm.latestValidChannelValue(4, 0);
  int in5 = ppm.latestValidChannelValue(5, 0);
  int in6 = ppm.latestValidChannelValue(6, 0);
  int in7 = ppm.latestValidChannelValue(7, 0);
  int in8 = ppm.latestValidChannelValue(8, 0);

  // SAFETY PROTOCOL
  if(in1 < 100){
    //Serial.println("OFF");
    analogWrite(AN1,0);
    analogWrite(AN2,0);
    analogWrite(AN3,0);
    analogWrite(AN4,0);
    analogWrite(AN5,0);
    analogWrite(AN6,0);
    analogWrite(AN7,0);
    analogWrite(AN8,0);

    regWrite(15,HIGH);
    regWrite(11,HIGH);
    regWrite(0,HIGH);
    for(int i = 1;i<11;i++){
      regWrite(i,LOW);
    }
    for(int i = 12;i<15;i++){
      regWrite(i,LOW);
    }

    safety = 1;

  }else{
    safety = 0;
  }

  if(safety == 0){

  if(in5 > 1969 && in8 > 1969){

    int gim1_panel = 0;
      int gim2_panel = 0;

    led_arr[7] = 1;
    int loco_panel = 0;
    int linear_panel = 0;

    if(in7 > 1300 && in7 < 1800){

      led_arr[5] = 1;
      led_arr[7] = 0;

      // led_arr[5] = 1;
      // led_arr[3] = 1;
      // // led_arr[1] = 1;
      // led_arr[13] = 1;

      automate = 1;
      

      if(Serial.available()){

        char c = Serial.read();
        //Serial.println(c);        

        if(c == '0'){
          //Left
          //Serial.println("Left");
          // analogWrite(AN1,255);
          linear_speed = 255;
          linear_dir = LOW;
          analogWrite(AN1,linear_speed);
          digitalWrite(EN1,linear_dir);
          // bldc_speed = 75;
          // bldc_dir = HIGH;
          analogWrite(AN2,68);
          digitalWrite(EN2,HIGH);
        }else if(c == '1'){
          //Stright
          //Serial.println("Straight");
          // analogWrite(AN1,0);
          // digitalWrite(EN1,HIGH);
          linear_speed = 0;
          linear_dir = HIGH;
          analogWrite(AN1,linear_speed);
          digitalWrite(EN1,linear_dir);
          // bldc_speed = 75;
          // bldc_dir = HIGH;
          analogWrite(AN2,68);
          digitalWrite(EN2,HIGH);


        }else if(c == '2'){
          //Right
          //Serial.println("Right");
          // analogWrite(AN1,255);
          // digitalWrite(EN1,HIGH);
          linear_speed = 255;
          linear_dir = HIGH;
          analogWrite(AN1,linear_speed);
          digitalWrite(EN1,linear_dir);
          // bldc_speed = 75;
          // bldc_speed = HIGH;
          analogWrite(AN2,68);
          digitalWrite(EN2,HIGH);
        }else if(c == '4'){
          //Stop
          //Serial.println("STOP");
          // analogWrite(AN1,0);
          // digitalWrite(EN1,HIGH);
          linear_speed = 0;
          linear_dir = HIGH;
          analogWrite(AN1,linear_speed); 
          digitalWrite(EN1,linear_dir);
          analogWrite(AN2,0);
          digitalWrite(EN2,HIGH);
          // bldc_speed = 0;
        }else{

          led_arr[13] = 1;
          //QR
          //Serial.println("QR");
          // analogWrite(AN1,0);
          // digitalWrite(EN1,HIGH);
          linear_speed = 0;
          linear_dir = HIGH;
          analogWrite(AN1,linear_speed);
          digitalWrite(EN1,linear_dir);
          // bldc_speed = 0;
          // delay(2000);
          // bldc_speed = 70;
          analogWrite(AN2,0);
          digitalWrite(EN2,HIGH);
          
          // delay(3000);
        }

      }
    
    }else{
      automate = 0;

    }

    // Serial.println("LOCOMOTION");

    //////////// LINEAR ACTUATOR AND BLDC MANUAL

    if(in4>1600){
      //go left
      //linear_speed = map(in4, 1300,970,0,255);
      linear_dir = HIGH;
      analogWrite(AN1,255);
      digitalWrite(EN1,linear_dir);
      
      led_arr[3] = 1;
      // led_arr[7] = 1;

      // Serial.print("Left: ");
      // Serial.println(linear_speed);

    }else if(in4<1300 && in4>100){

      led_arr[3] = 1;
      // led_arr[7] = 1;
      //go right
      //linear_speed = map(in4, 1600,2000,0,255);
      linear_dir = LOW;
      analogWrite(AN1,255);
      digitalWrite(EN1,linear_dir);
      // Serial.print("Right: ");
      // Serial.println(linear_speed);

    }else{
      //stop
      if(in6>1500){
        dist_cm = dist();
        //Serial.println(dist_cm);
        center();
      }
      else if(automate == 1){

      }else{
        // led_arr[2] = 0;
        // loco_man = 1;
        linear_panel = 1;
        linear_speed = 0;
        analogWrite(AN1,linear_speed);
        digitalWrite(EN1,linear_dir);
        // Serial.println("Linear STOP");
      }
    }
    
    //FAST locomotion
    if (in2 < 1250 && in2 > 100){
      digitalWrite(49,LOW);
      digitalWrite(51,LOW);
      digitalWrite(53,LOW);
      analogWrite(10, 0);

      led_arr[3] = 1;
      // led_arr[7] = 1;

      digitalWrite(EN2,bldc_dir);
      // delay(10);
      //go back
      bldc_speed = map(in2, 1250,1000,140,210);
      bldc_dir = LOW;
      analogWrite(AN2,bldc_speed);
      
      // Serial.print("Back: ");
      // Serial.println(bldc_speed);
      

    }else if(in2>1750){
      digitalWrite(49,LOW);
      digitalWrite(51,LOW);
      digitalWrite(53,LOW);
      analogWrite(10, 0);

      led_arr[3] = 1;
      // led_arr[7] = 1;
      
      digitalWrite(EN2,bldc_dir);
      // delay(10);
      //go front
      bldc_speed = map(in2, 1750,2000,140,210);
      bldc_dir = HIGH;
      analogWrite(AN2,bldc_speed);
      // Serial.print("Front: ");
      // Serial.println(bldc_speed);

    }else{
      //stop
      if(automate == 1){
        
      }else{
        // led_arr[2] = 1;
        loco_panel = 1;
        bldc_speed = 0;
        analogWrite(AN2,bldc_speed);
        digitalWrite(EN2,bldc_dir);
        Serial.println("BLDC STOP");

        // int break_speed = map(in2, 1400,1500,0,90);
        // digitalWrite(49,HIGH);
        // analogWrite(10,break_speed);

        // //int break_speed = map(in2, 1700,2000,140,210);
        if(in2>=1500 && in2 > 100){
          int break_speed = map(in2, 1750,1500,0,196);
          digitalWrite(49,HIGH);
          digitalWrite(51,HIGH);
          digitalWrite(53,HIGH);
          analogWrite(10,break_speed);


        }else if(in2<1500){
          int break_speed = map(in2, 1250,1500,0,196);
          digitalWrite(49,HIGH);
          digitalWrite(51,HIGH);
          digitalWrite(53,HIGH);
          analogWrite(10,break_speed);
        }
      }
    }

    ///////// GIMBLE MANUAL MOTION
    //motor1
    if(in7>1700){

      led_arr[7] = 0;
      led_arr[8] = 1;

      

      if (in1 < 1300 && in1 > 100){
        //go left

        led_arr[1] = 1;

        linear_speed = 0;
        gimble1_speed = map(in1, 1300,970,0,70);
        gimble1_dir = HIGH;

        if(digitalRead(gimble_left)==0){
          gimble1_speed = 0;
        }

        analogWrite(AN7,gimble1_speed);
        digitalWrite(EN7,gimble1_dir);

      }else if(in1>1600){

        led_arr[1] = 1;
        //go right
        linear_speed = 0;
        gimble1_speed = map(in1, 1600,2000,0,70);
        gimble1_dir = LOW;
        // Serial.print("Gimble Right: ");
        // Serial.println(gimble1_speed);

        if(digitalRead(gimble_right)==0){
          gimble1_speed = 0;
        }

        analogWrite(AN7,gimble1_speed);
        digitalWrite(EN7,gimble1_dir);

      }else{
        //stop
        led_arr[10] = 1;
        gimble1_speed = 0;
        gim1_panel = 1;
        // Serial.println("Gimble 1 STOP");
        analogWrite(AN7,gimble1_speed);
        digitalWrite(EN7,gimble1_dir);
      }

      //motor2
      if(in3>1600){

        led_arr[1] = 1;
        //go back
        bldc_speed = 0;
        gimble2_speed = map(in3, 1300,1000,0,70);
        gimble2_dir = LOW;
        analogWrite(AN8,gimble2_speed);
        digitalWrite(EN8,gimble2_dir);

      }else if(in3 < 1300 && in3 > 100){

        led_arr[1] = 1;
        //go front
        bldc_speed = 0;
        gimble2_speed = map(in3, 1600,2000,0,70);
        gimble2_dir = HIGH;
        analogWrite(AN8,gimble2_speed);
        digitalWrite(EN8,gimble2_dir);

      }else{
        //stop
        led_arr[10] = 1;
        gimble2_speed = 0;
        gim2_panel = 1;
        analogWrite(AN8,gimble2_speed);
        digitalWrite(EN8,gimble2_dir);
      }

      


    }else if(in7 < 1300){
    }

    /////////// ROTATE and CLICK MANUAL

    if(in6>1600 && in7>1300 && in7 < 1700){
        // Serial.println("Rotate");
        // regWrite(10, HIGH);
        led_arr[13] = 1;
        rotate();
    }  

    if(in6>1600 && in7>1700){
        // led_arr[10] = 1;
        // regWrite(10, HIGH);
        // Serial.println("Rotate");
        //rotate();
        Serial.println("X");
        Serial.println("X");
        //Serial.println("X");
        delay(1000);
    }



    m4_speed = 0;
    m3_speed = 0;
    m2_speed = 0;
    m1_speed = 0;
    analogWrite(AN3,m4_speed);
    analogWrite(AN4,m3_speed);
    analogWrite(AN5,m2_speed);
    analogWrite(AN6,m1_speed);

    if(loco_panel && linear_panel){
      led_arr[2] = 1;
    }else{
      led_arr[2] = 0;
    }

    if(gim1_panel && gim2_panel){
      led_arr[10] = 1;
    }else{
      led_arr[10] = 0;
    }

    if(led_arr[8] == 1){
      led_arr[2] = 0;
      led_arr[3] = 0;
    }

    if(led_arr[5]==1){
      led_arr[3] = 0;
    }

  }

  //turn on ARM
  else if(in5 < 1069 && in8 < 1069 && in5 > 100 && in8 > 100){

    int m1_panel = 0;
    int m2_panel = 0;
    int m3_panel = 0;
    int m4_panel = 0;

    led_arr[9] = 1;

    ////// ARM MANUAL - OIL, WATER, AUTO

    if(in7 < 1300 && in7 > 100){ 
      arm_automate = 0;
      
      if(in6 > 1500 && in8 < 1500){
        myservo.write(0);
        delay(1000);
        myservo.write(150);
        delay(1000);
      }
      else{
        myservo.write(0);
      }

    }else if(in7 < 1700 && in8 < 1500){
      arm_automate = 0;

      if(in6 > 1500){
        digitalWrite(water_pump,HIGH);
        digitalWrite(bosch,HIGH);
      }
      else{
        digitalWrite(water_pump,LOW);
        digitalWrite(bosch,LOW);
      }

    }else{
      arm_automate = 1;
      int d_startProcess = LOW;

      if(in6 > 1500 && in8 < 1500){
        d_startProcess = HIGH;
      }

      //////////////// ARM AUTOMATION
      led_arr[12] = 1;
      led_arr[9] = 0;

            
        //water check code body 
        d_tankFull = digitalRead(i_floatSensorNC);    
        d_tankEmpty = digitalRead(i_floatSensorNO);   
        d_tankStatus = waterCheck(d_tankFull, d_tankEmpty);

        int d_topLimit = digitalRead(i_topLimitL) && digitalRead(i_topLimitR) ;
        int d_bottomLimit = digitalRead(i_bottomLimitL) && digitalRead(i_bottomLimitR);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////


        // Only proceed if the water tank level is obtimum
        if (d_tankStatus != 0){
          // Finite State Machine
          switch(currentState) {
            case minusState: //Minus State (checking)
              if (d_startProcess == 1) {currentState = zeroState;}
              else {analogWrite(analogMotor1, 0);}
            break;

            case zeroState: //Initial State (arm goes down)
              if (d_bottomLimit == 1) {
                // Serial.println("Initial State (arm goes down)");
                digitalWrite(digitalMotor1, LOW);
                analogWrite(analogMotor1, ballScrewMotorSpeed);
              }
              else {
                analogWrite(analogMotor1, 0);
                currentState = oneState;
              }
            break;

            case oneState:  //First State (arm goes up and sprays air)
              // Serial.println("First State (arm goes up and sprays air)");
              if (d_topLimit == 1) {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(1);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);

                digitalWrite(digitalMotor1, HIGH);
                analogWrite(analogMotor1, ballScrewMotorSpeed);        
              }
              else {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                analogWrite(analogMotor1, 0);
                currentState = twoState;     
              }
            break;

            case twoState: // Second State (arm goes down and sprays water)
              // Serial.println("Second State (arm goes down and sprays water)");
              if (d_bottomLimit == 1) {
                d_boschRelayStatus = boschCheck(1);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(1);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                digitalWrite(digitalMotor1, LOW);
                analogWrite(analogMotor1, ballScrewMotorSpeed);        
              }
              else {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                analogWrite(analogMotor1, 0);
                currentState = threeState;     
              }
            break;

            case threeState: // Third State (arm goes up and sprays soap)
              if (d_topLimit == 1) {
                d_boschRelayStatus = boschCheck(1);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(1);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(1);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                digitalWrite(digitalMotor1, HIGH);
                analogWrite(analogMotor1, ballScrewMotorSpeed);        
              }
              else {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                analogWrite(analogMotor1, 0);
                currentState = fourState;     
              }
            break;

            case fourState: // Fourth State (arm goes down and sprays water)
              if (d_bottomLimit == 1) {
                d_boschRelayStatus = boschCheck(1);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(1);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                digitalWrite(digitalMotor1, LOW);
                analogWrite(analogMotor1, ballScrewMotorSpeed);        
              }
              else {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                analogWrite(analogMotor1, 0);
                currentState = fiveState;     
              }
            break;

            case fiveState: // Fifth State (arm goes up and sprays air)
              if (d_topLimit == 1) {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(1);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                
                digitalWrite(digitalMotor1, HIGH);
                analogWrite(analogMotor1, ballScrewMotorSpeed);        
              }
              else {
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
                    
                analogWrite(analogMotor1, 0);
                currentState = minusState; 
                d_startProcess = LOW;    
              }
            break;
          }
        }
        else {
          // All motors are turned off
                d_boschRelayStatus = boschCheck(0);                             digitalWrite(o_boschRelay, d_boschRelayStatus);
                d_pneumaticRelayStatus = pneumaticCheck(0);                     digitalWrite(o_pneumaticRelay, d_pneumaticRelayStatus);
                d_soapPumpRelayStatus = soapPumpCheck(0);                       digitalWrite(o_soapPumpRelay, d_soapPumpRelayStatus);
                d_pumpRelayStatus = pumpCheck(0);                               digitalWrite(o_pumpRelay, d_pumpRelayStatus);
          analogWrite(analogMotor1, 0);
        }


    }

    /////////// ARM AUTOMATION END

    ///////// ARM MOTION 

    //motor1
    if(arm_automate == 0){
      
      if (in3 < 1450 && in3 > 100){

        led_arr[4] = 1;
        m1_speed = map(in3, 1450,1000,20,150);
        m1_dir = LOW;

        if((digitalRead(i_bottomLimitR)==0) || (digitalRead(i_bottomLimitL)==0)){
          m1_speed = 0;
        }
        analogWrite(AN6,m1_speed);
        digitalWrite(EN6,m1_dir);

      }else if(in3>1550){
        //go right
        led_arr[4] = 1;
        m1_speed = map(in3, 1550,2000,20,150);
        m1_dir = HIGH;

        if((digitalRead(i_topLimitR)==0) || (digitalRead(i_topLimitL)==0)){
          m1_speed = 0;
        }

        analogWrite(AN6,m1_speed);
        digitalWrite(EN6,m1_dir);

      }else{
        //stop
        m1_speed = 0;
        m1_panel = 1;
        analogWrite(AN6,m1_speed);
        digitalWrite(EN6,m1_dir);
      }
    }
      if (in2 < 1300 && in2 > 100){

        led_arr[4] = 1;
        m2_speed = map(in2, 1450,1000,0,80);
        m2_dir = HIGH;
        analogWrite(AN5,m2_speed);
        digitalWrite(EN5,m2_dir);

      }else if(in2>1600){

        led_arr[4] = 1;
        //go right
        m2_speed = map(in2, 1550,2000,0,80);
        m2_dir = LOW;
        analogWrite(AN5,m2_speed);
        digitalWrite(EN5,m2_dir);

      }else{
        //stop
        m2_speed = 0;
        m2_panel = 1;
        analogWrite(AN5,m2_speed);
        digitalWrite(EN5,m2_dir);
      }

    // }

      //motor3
      if (in1 > 1600){

        led_arr[4] = 1;
        m3_speed = map(in1, 1550,2000,20,80);
        m3_dir = HIGH;
        analogWrite(AN4,m3_speed);
        digitalWrite(EN4,m3_dir);

      }else if(in1<1300 && in1 > 100){
        //go right
        led_arr[4] = 1;
        m3_speed = map(in1, 1450,1000,20,80);
        m3_dir = LOW;
        analogWrite(AN4,m3_speed);
        digitalWrite(EN4,m3_dir);

      }else{
        //stop
        m3_speed = 0;
        m3_panel = 1;
        analogWrite(AN4,m3_speed);
        digitalWrite(EN4,m3_dir);
      }

      //motor4 - lead screw
      if (in4 < 1300 && in4 > 100){

        led_arr[4] = 1;
        m4_speed = map(in4, 1450,1000,30,80);
        m4_dir = HIGH;
        if(i_bottomLimitR == 1 || i_bottomLimitL == 1){
          m4_speed = 0;
        }
        analogWrite(AN3,m4_speed);
        digitalWrite(EN3,m4_dir);

      }else if(in4>1600){

        led_arr[4] = 1;
        //go right
        m4_speed = map(in4, 1550,2000,30,80);
        m4_dir = LOW;
        if(i_topLimitR == 1 || i_topLimitL == 1){
          m4_speed = 0;
        }
        analogWrite(AN3,m4_speed);
        digitalWrite(EN3,m4_dir);

      }else{
        //stop
        m4_speed = 0;
        m4_panel = 1;
        analogWrite(AN3,m4_speed);
        digitalWrite(EN3,m4_dir);
      }

    //// ARM MOTION END
    if(m1_panel && m2_panel && m3_panel && m4_panel ){
      led_arr[6] = 1;
    }else{
      led_arr[6] = 0;
    }

    if(led_arr[12]==1){
      led_arr[6] = 0;
      led_arr[4] = 0;
    }

  }
  else if(in5 > 1400 && in5 < 1600 && in8 < 1069 ){

    Serial.println("ARM AIR");

    if(in6>1500){
      digitalWrite(pneumatic, HIGH);
    }
    

  }
  else if(in5 > 1400 && in5 < 1600 && in8 > 1869 ){

    if(in4>1600){
      //go left
      linear_dir = HIGH;
      analogWrite(AN1,255);
      digitalWrite(EN1,linear_dir);
      
      led_arr[3] = 1;

    }else if(in4<1300 && in4>100){

      led_arr[3] = 1;
      linear_dir = LOW;
      analogWrite(AN1,255);
      digitalWrite(EN1,linear_dir);

    }else{
      //stop
      if(in6>1500){
        dist_cm = dist();
        center();
      }
      else if(automate == 1){

      }else{
        analogWrite(AN1,linear_speed);
        digitalWrite(EN1,linear_dir);
      }
    }

    Serial.println("SLOW locomotion");
    led_arr[7] = 1;

    if(in2<1450){
      digitalWrite(49,LOW);
      digitalWrite(51,LOW);
      digitalWrite(53,LOW);
      analogWrite(10, 0);

      led_arr[3] = 1;

      digitalWrite(EN2,bldc_dir);
      // delay(10);
      //go back
      bldc_speed = map(in2, 1450,1000,120,210);
      bldc_dir = LOW;
      analogWrite(AN2,bldc_speed);
      
      Serial.print("Back: ");
      Serial.println(bldc_speed);
      

    }else if(in2>1550){
      digitalWrite(49,LOW);
      digitalWrite(51,LOW);
      digitalWrite(53,LOW);
      analogWrite(10, 0);

      led_arr[3] = 1;
      
      digitalWrite(EN2,bldc_dir);
      bldc_speed = map(in2, 1550,2000,120,210);
      bldc_dir = HIGH;
      analogWrite(AN2,bldc_speed);
      Serial.print("Front: ");
      Serial.println(bldc_speed);

    }else{
      led_arr[2] = 1;
      digitalWrite(49,HIGH);
      digitalWrite(51,HIGH);
      digitalWrite(53,HIGH);
      analogWrite(10, 0);
      analogWrite(AN2,0);
    }
    
  }
  else{

    analogWrite(AN1,0);
    analogWrite(AN2,0);
    analogWrite(AN3,0);
    analogWrite(AN4,0);
    analogWrite(AN5,0);
    analogWrite(AN6,0);
    analogWrite(AN7,0);
    analogWrite(AN8,0);

    digitalWrite(EN3,m4_dir);
    digitalWrite(EN4,m3_dir);
    digitalWrite(EN5,m2_dir);
    digitalWrite(EN6,m1_dir);

  }

  for(int prat=0; prat<sizeof(led_arr); prat++){
    if(led_arr[prat] == 1){
      regWrite(prat, HIGH);
    }else{
      regWrite(prat, LOW);
    }
  }

  }

}



void regWrite(int pin, bool state){
  //Determines register
  int reg = pin / 8;
  //Determines pin for actual register
  int actualPin = pin - (8 * reg);

  //Begin session
  digitalWrite(latchPin, LOW);

  for (int i = 0; i < numOfRegisters; i++){
    //Get actual states for register
    byte* states = &registerState[i];

    //Update state
    if (i == reg){
      bitWrite(*states, actualPin, state);
    }

    //Write
    shiftOut(dataPin, clockPin, MSBFIRST, *states);
  }

  //End session
  digitalWrite(latchPin, HIGH);
}
