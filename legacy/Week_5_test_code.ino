//outputs to motor driver
int motorAPhase = 36; // P6.6 green wire
int motorLPWM = 39; // P2.6 yellow wire
int motorBPhase = 38; // P2.4 green wire
int motorRPWM = 37; // P5.6 yellow wire
 
int FRIR = A14;
int RIR =A13;
int CIR = A11;
int LIR = A9;
int FLIR = A6;
const int sensorPin = A1;
//LED states | true = white | false = black
const int threshold = 600;
const int Speed = 230;
int timer = millis();
int state = 0;
int FLIR_Average;
 int LIR_Average;
 int CIR_Average;
 int RIR_Average;
 int FRIR_Average;

 int position1 = 0;
int destination = 0;

bool StartFlag = 1;
//wifi start
#include <WiFi.h>
#include <WiFiClient.h>
#include <SPI.h>
#define BUFSIZE 512

// wifi details
String postBody = "";
char ssid[] = "REDACTED";
char password[] = "REDACTED";
WiFiClient client;


// server details
char server[] = "REDACTED";
int port = 8081;

int NewDir[8][8] = {
  {0,0,0,0,1,0,0,1},
  {1,1,1,0,0,0,1,0},
  {1,1,1,0,1,1,1,1},
  {1,0,1,1,0,0,1,0},
  {0,1,0,1,1,1,1,0},
  {1,1,1,1,1,1,1,1},
  {4,2,4,4,1,2,2,1},
  {0,3,1,5,5,2,2,1}
};

struct Node{
  int item;
  Node *CW;
  Node *center;
  Node *ACW;
};

struct Node position[8];


// the setup routine runs once when you press reset:
void setup() {
 Serial.begin(9600);
 for(int i = 0; i < 8; i++){
  position[i].item = i;
 }
 
 position[0].CW = &position[4];
 position[0].ACW = &position[6];

 position[1].CW = &position[6];
 position[1].ACW = &position[7];

 position[2].CW = &position[6];
 position[2].ACW = &position[3];

 position[3].CW = &position[2];
 position[3].ACW = &position[7];

 position[4].CW = &position[7];
 position[4].ACW = &position[0];

 position[5].CW = &position[4];
 position[5].ACW = &position[6];

 position[6].CW = &position[0];
 position[6].ACW = &position[2];
 position[6].center = &position[1];

position[7].CW = &position[3];
 position[7].ACW = &position[4];
position[7].center = &position[1];
 connectToWiFi();
 timer = millis();
 if(millis() - timer > 100){
  ReadSensors();
 }
 state = GetState();
 FollowLine();
 }
// the loop routine runs over and over again continuously:

int newDir = 0;
int counter = 0;
int direction = 0; //false = acw,true = cw

struct Node *curr = &position[0];
int prev;
void loop() {
  
  ReadSensors();
 // ever 0.05 seconds checks the state of the IR average and moves the motors in a specific way to make the CIR sensor always above the white line
 if(millis() - timer > 30){
  timer = millis();
  state = GetState();
  switch(state){
    case 0:
    MoveBack();
      while(!(CheckState(FLIR_Average) || CheckState(LIR_Average) || CheckState(CIR_Average) || CheckState(RIR_Average) || CheckState(FRIR_Average))){
          ReadSensors();
          if(millis() - timer > 40){
            timer = millis();
          state = GetState();
          }
        }
      timer = millis()-200;
      break;
    case 1:
      TurnRight4();
      break;
    case 2:
      TurnRight2();
      break;
    case 3:
      TurnRight4();     
      break;
    case 4:
      MoveForward();     
      break;
    case 5:
      MoveForward();
      break;
    case 6:
      TurnRight1();
      break;
    case 7:
      TurnRight4();
      break;
    case 8:
      TurnLeft2();
      break;
    case 9:
      MoveForward();
      break;
    case 10:
      MoveForward();
      break;
    case 11:
      MoveForward();
      break;
    case 12: 
      TurnLeft1();   
      break;
    case 13:
      MoveForward();
      break;
    case 14:
      MoveForward();    
      break;
    case 15:
      TurnRight3();
      break;
    case 16:
      TurnLeft4();
      break;
    case 17:
      MoveForward();
      break;
    case 18:
      MoveForward();
      break;
    case 19:
      MoveForward();
      break;
    case 20:
      MoveForward();
      break;
    case 21:
      MoveForward();
      break;
    case 22:
      MoveForward();
      break;
    case 23:
      MoveForward();
      break;
    case 24:
    TurnLeft4();
      break;
    case 25:
       MoveForward();
      break;
    case 26:
     MoveForward();
      break;
    case 27:
     MoveForward();
      break;
    case 28:
    TurnRight3();
      break;
    case 29:
    MoveForward();
      break;
    case 30:
    TurnLeft3();
      break;
    case 31:

    if(StartFlag == 0){
      if(((int(curr->item) == 6) || (int(curr->item) == 7)) && (destination == 1)){
        prev = curr->item;
        curr =curr->center;
      }
      else if(direction == 0){
        prev = curr->item;
        curr = curr->ACW;
    }
    else if(direction == 1){
      prev = curr->item;
      curr = curr->CW;
    }
      
    }
    StartFlag = 0;
    Serial.println("Current position: " + String(curr->item));
    
    if(destination == int(curr->item)){
      Stop();
      position1 = destination;
      destination = UpdatePath(position1);
    }
    
    
    if((NewDir[curr->item][destination] == 2)){

      if((prev == 1) && (destination == 5)){
        GoTo5();
      }
      if((prev == 0) || prev == 4){
        TurnLeft90();
        direction = 0;
      }else{
        TurnRight90();
        direction = 0;
      }
      if(destination == 5){
        timer = millis();
        GoTo5();
      }
    }else if((NewDir[curr->item][destination] == 3)){
      if(prev == 3){
        TurnLeft90();
        direction = 1;
      }else{
        TurnRight90();
        direction = 1;
      }
    }
    else if((NewDir[curr->item][destination] == 4) && (prev ==1) ){
      if(destination == 0){
        TurnRight90();
        direction = 1;
      }else{
        TurnLeft90();
        direction = 0;
      }
    }
    else if((NewDir[curr->item][destination] == 5) && (prev ==1)){
      
        if(destination == 3){
        TurnRight90();
        direction = 1;
      }else{
        TurnLeft90();
        direction = 0;
      }
    }
    else if(direction^NewDir[position1][destination]){
          TurnACW180();
          direction = NewDir[position1][destination];
        }
    WaitTillOffWhite();
    timer = millis();
      break;                  
  
 }
 }
 }

 void ReadSensors(){
    FLIR_Average = runningAverageFL(analogRead(FLIR));
    LIR_Average = runningAverageL(analogRead(LIR));
    CIR_Average = runningAverageC(analogRead(CIR));
    RIR_Average = runningAverageR(analogRead(RIR));
    FRIR_Average = runningAverageFR(analogRead(FRIR));  
 }
 
  long runningAverageR(int newValue) {
    #define BUFFER_LENGTH 100
    static int bufferR[BUFFER_LENGTH];
    static int currentIndex = 0;
    static int sum = 0;
    static int count = 0;

    sum -= bufferR[currentIndex];
    sum += newValue;
    bufferR[currentIndex] = newValue;
    currentIndex = (currentIndex + 1) % BUFFER_LENGTH;
    count = (count < BUFFER_LENGTH) ? count + 1 : BUFFER_LENGTH;

    return (long)sum / count;
}
long runningAverageFL(int newValue) {
    static int bufferFL[BUFFER_LENGTH];
    static int currentIndex = 0;
    static int sum = 0;
    static int count = 0;

    sum -= bufferFL[currentIndex];
    sum += newValue;
    bufferFL[currentIndex] = newValue;
    currentIndex = (currentIndex + 1) % BUFFER_LENGTH;
    count = (count < BUFFER_LENGTH) ? count + 1 : BUFFER_LENGTH;

    return (long)sum / count;
}
long runningAverageL(int newValue) {
    static int bufferL[BUFFER_LENGTH];
    static int currentIndex = 0;
    static int sum = 0;
    static int count = 0;

    sum -= bufferL[currentIndex];
    sum += newValue;
    bufferL[currentIndex] = newValue;
    currentIndex = (currentIndex + 1) % BUFFER_LENGTH;
    count = (count < BUFFER_LENGTH) ? count + 1 : BUFFER_LENGTH;

    return (long)sum / count;
}
long runningAverageC(int newValue) {
    static int bufferC[BUFFER_LENGTH];
    static int currentIndex = 0;
    static int sum = 0;
    static int count = 0;

    sum -= bufferC[currentIndex];
    sum += newValue;
    bufferC[currentIndex] = newValue;
    currentIndex = (currentIndex + 1) % BUFFER_LENGTH;
    count = (count < BUFFER_LENGTH) ? count + 1 : BUFFER_LENGTH;

    return (long)sum / count;
}
long runningAverageFR(int newValue){
    static int bufferFR[BUFFER_LENGTH];
    static int currentIndex = 0;
    static int sum = 0;
    static int count = 0;

    sum -= bufferFR[currentIndex];
    sum += newValue;
    bufferFR[currentIndex] = newValue;
    currentIndex = (currentIndex + 1) % BUFFER_LENGTH;
    count = (count < BUFFER_LENGTH) ? count + 1 : BUFFER_LENGTH;

    return (long)sum / count;
}
//checks whether the ir is above white line or black based on the threshold set
 boolean CheckState(long IR_Average){
  if(IR_Average < threshold){
    return true;
  }
  return false;
 }
 //Moves the robot forward
 void MoveForward(){
  digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, Speed);
  analogWrite(motorRPWM, Speed);
 }
  void MoveBack(){
  digitalWrite(motorBPhase, HIGH);
  digitalWrite(motorAPhase, HIGH);
  analogWrite(motorLPWM, 50);
  analogWrite(motorRPWM, 50);
 }

 //turns the robot left
void TurnLeft1(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, Speed-25);
  analogWrite(motorRPWM, Speed);
}
void TurnLeft2(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, Speed-40);
  analogWrite(motorRPWM, Speed);
}
void TurnLeft3(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, 50);
  analogWrite(motorRPWM, 100);
}

//turns the robot with a smaller radius of rotation right
void TurnRight1(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, Speed);
  analogWrite(motorRPWM, Speed-25);
}
void TurnRight2(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, Speed);
  analogWrite(motorRPWM, Speed-40);
}
void TurnRight3(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, 100);
  analogWrite(motorRPWM, 30);
}
void TurnRight4(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, 240);
  analogWrite(motorRPWM, 50);
}
void TurnLeft4(){
   digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  analogWrite(motorLPWM, 50);
  analogWrite(motorRPWM, 240);
}

//turns the robot with a smaller radius of rotation left
//Stops the vehicle from moving
void Stop(){
  analogWrite(motorLPWM, 0);
  analogWrite(motorRPWM, 0);
}
int GetState(){
  int FLIR_state = CheckState(FLIR_Average);
  int LIR_state = CheckState(LIR_Average);
  int CIR_state = CheckState(CIR_Average);
  int RIR_state = CheckState(RIR_Average);
  int FRIR_state = CheckState(FRIR_Average);
  return (FLIR_state * 16) + (LIR_state * 8) + (CIR_state * 4) + (RIR_state*2) + FRIR_state;
}


void TurnACW180(){
  analogWrite(motorLPWM, 200);
  analogWrite(motorRPWM, 200);
  digitalWrite(motorBPhase, HIGH);
  digitalWrite(motorAPhase, LOW);
  int timerr = millis();
      while(millis() - timerr < 600){
       ReadSensors();
        }
 
}

void WaitTillOffWhite(){
  while(CheckState(FLIR_Average) && CheckState(LIR_Average) && CheckState(CIR_Average) && CheckState(RIR_Average) && CheckState(FRIR_Average)){
          ReadSensors();
          timer = millis();
          if(millis() - timer > 40){
            timer = millis();
          state = GetState();
          }
        }
        timer = millis();
}





//wifi start

void connectToWiFi() {
  Serial.print("Connecting to network: ");
  Serial.print(ssid);
  Serial.flush();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    Serial.flush();
    delay(300);
  }
  Serial.println("Connected");
  Serial.print("Obtaining IP address");
  Serial.flush();
  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");
    Serial.flush();
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}


int UpdatePath(int pos){
  if (client.connect(server, port)) {
    Serial.println("Connecting to server");
  }
 
  // post body
  postBody = "position=";
  postBody = postBody += pos;
 Serial.println(postBody);
 
  // send post request and headers
  client.println("POST /api/arrived/REDACTED HTTP/1.1");
  client.println("Content-Type: application/x-www-form-urlencoded");  
  client.print("Content-Length: ");
  client.println(postBody.length());
  client.println("Connection: close");
  client.println();
 
  // send post body
  client.println(postBody);
  int timer6 = millis();
        while(millis() - timer6 < 1000){        
        }
        
  char buffer[BUFSIZE];
  memset(buffer, 0, BUFSIZE);
  client.readBytes(buffer, BUFSIZE);
  String response(buffer);
  Serial.print("The response is: ");
  Serial.println(response);
 
  int index = response.indexOf("\r\n\r\n") + 4;
  int endIndex = response.indexOf("\r", index);
  String positionIndex = response.substring(index, endIndex);
 
   
  Serial.println("Destination: ");
  Serial.println(positionIndex.toInt());
  return positionIndex.toInt();
}

void FollowLine(){
  while(state != 31){
     ReadSensors();
 // ever 0.05 seconds checks the state of the IR average and moves the motors in a specific way to make the CIR sensor always above the white line
 if(millis() - timer > 20){
  timer = millis();
  state = GetState();
  switch(state){
    case 0:
    MoveBack();
      while(!(CheckState(FLIR_Average) || CheckState(LIR_Average) || CheckState(CIR_Average) || CheckState(RIR_Average) || CheckState(FRIR_Average))){
          ReadSensors();
          if(millis() - timer > 40){
            timer = millis();
          state = GetState();
          }
        }
      timer = millis()-100;
      break;
    case 1:
      TurnRight4();
      break;
    case 2:
      TurnRight2();
      break;
    case 3:
      TurnRight4();     
      break;
    case 4:
      MoveForward();     
      break;
    case 5:
      MoveForward();
      break;
    case 6:
      TurnRight1();
      break;
    case 7:
      TurnRight4();
      break;
    case 8:
      TurnLeft2();
      break;
    case 9:
      MoveForward();
      break;
    case 10:
      MoveForward();
      break;
    case 11:
      MoveForward();
      break;
    case 12: 
      TurnLeft1();   
      break;
    case 13:
      MoveForward();
      break;
    case 14:
      MoveForward();    
      break;
    case 15:
      TurnRight3();
      break;
    case 16:
      TurnLeft4();
      break;
    case 17:
      MoveForward();
      break;
    case 18:
      MoveForward();
      break;
    case 19:
      MoveForward();
      break;
    case 20:
      MoveForward();
      break;
    case 21:
      MoveForward();
      break;
    case 22:
      MoveForward();
      break;
    case 23:
      MoveForward();
      break;
    case 24:
    TurnLeft4();
      break;
    case 25:
       MoveForward();
      break;
    case 26:
     MoveForward();
      break;
    case 27:
     MoveForward();
      break;
    case 28:
    TurnRight3();
      break;
    case 29:
    MoveForward();
      break;
    case 30:
    TurnLeft3();
      break;
    case 31:
    MoveForward();  
      break;                  
    }
  }
}
timer = millis();
}
void TurnRight90(){
  analogWrite(motorLPWM, 200);
  analogWrite(motorRPWM, 0);
  digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  int timerr = millis();
  while(millis() - timerr < 500){
    ReadSensors();
  }  
}

void TurnLeft90(){
   analogWrite(motorLPWM, 0);
  analogWrite(motorRPWM, 200);
  digitalWrite(motorBPhase, LOW);
  digitalWrite(motorAPhase, LOW);
  int timerr = millis();
      while(millis() - timerr < 500){
         ReadSensors();
        }
}
int Flag = 1;
float Valuue;
void GoTo5(){
  while(true){

       ReadSensors();
       state = GetState();
       
 // ever 0.05 seconds checks the state of the IR average and moves the motors in a specific way to make the CIR sensor always above the white line
 if(millis() - timer > 50){
  timer = millis();
  state = GetState();
  switch(state){
    case 0:
    MoveForward();
    while(true){
      timer = millis();
      while(millis() - timer < 100){
        Valuue = ReadDistanceSensor();
      }
      
      if(Valuue < 10){
        analogWrite(motorLPWM, 80);
        analogWrite(motorRPWM, 80);
        digitalWrite(motorBPhase, LOW);
        digitalWrite(motorAPhase, LOW);
        Valuue = ReadDistanceSensor();        
              if(Valuue < 5){
        timer = millis();
        while(millis() - timer < 450){  
        }
        Stop();
        destination = UpdatePath(5);
        while(true){
        }
      }
      }
    }
 
      break;
    case 1:
      TurnRight4();
      break;
    case 2:
      TurnRight2();
      break;
    case 3:
      TurnRight4();     
      break;
    case 4:
      MoveForward();     
      break;
    case 5:
      MoveForward();
      break;
    case 6:
      TurnRight1();
      break;
    case 7:
      TurnRight4();
      break;
    case 8:
      TurnLeft2();
      break;
    case 9:
      MoveForward();
      break;
    case 10:
      MoveForward();
      break;
    case 11:
      MoveForward();
      break;
    case 12: 
      TurnLeft1();   
      break;
    case 13:
      MoveForward();
      break;
    case 14:
      MoveForward();    
      break;
    case 15:
      TurnRight3();
      break;
    case 16:
      TurnLeft4();
      break;
    case 17:
      MoveForward();
      break;
    case 18:
      MoveForward();
      break;
    case 19:
      MoveForward();
      break;
    case 20:
      MoveForward();
      break;
    case 21:
      MoveForward();
      break;
    case 22:
      MoveForward();
      break;
    case 23:
      MoveForward();
      break;
    case 24:
    TurnLeft4();
      break;
    case 25:
       MoveForward();
      break;
    case 26:
     MoveForward();
      break;
    case 27:
     MoveForward();
      break;
    case 28:
    TurnRight3();
      break;
    case 29:
    MoveForward();
      break;
    case 30:
    TurnLeft3();
      break;
    case 31:
    MoveForward();  
      break;                  
    }
  }
  }
}
#define BUFFER_LENGTH2 50
long runningAverageDistance(long newValue){
    static long bufferDistance[BUFFER_LENGTH2];
    static int currentIndex = 0;
    static long sum = 0;
    static int count = 0;

    sum -= bufferDistance[currentIndex];
    sum += newValue;
    bufferDistance[currentIndex] = newValue;
    currentIndex = (currentIndex + 1) % BUFFER_LENGTH2;
    count = (count < BUFFER_LENGTH2) ? count + 1 : BUFFER_LENGTH2;

    return (long)sum / count;
}

  float ReadDistanceSensor(){
  float volts = analogRead(sensorPin)*0.0048828125;  // value from sensor * (5/1024)
  float distance = 13*pow(volts, -1); // worked out from datasheet graph

  return distance;
  }
