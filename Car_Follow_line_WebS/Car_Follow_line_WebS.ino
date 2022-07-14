#include <esp_timer.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

// SSID & Password
const char* ssid = "ESP32WebServer";
const char* password = "123456789";

// IP Address details
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

//WebServer server(80);
AsyncWebServer server(80);

#define FORWARD     0x01        //0b01
#define BACKWARD    0x02        //0b10
#define BRAKE0      0x00        //0b00
#define BRAKE1      0x03        //0b11

#define pin_motor0A  12         //Pin Utilizado para el control del Motor 0 EnA
#define pin_motor0B  21         //Pin utilizado para el control del Motor 0 EnB
#define pin_motor1A  22         //Pin utilizado para el control del Motor 1 EnA
#define pin_motor1B  4          //Pin utilizado para el control del Motor 1 EnB
#define pin_motor2A  23         //Pin utilizado para el control del Motor 2 EnA
#define pin_motor2B  2          //Pin utilizado para el control del Motor 2 EnB
#define pin_motor3A  19         //Pin utilizado para el control del Motor 3 EnA
#define pin_motor3B  18         //Pin utilizado para el control del Motor 3 EnB
#define pin_encoder0  25        //Pin de entrada del encoder 0 asociado al motor 0
#define pin_encoder1  27        //Pin de entrada del encoder 1 asociado al motor 1
#define pin_encoder2  35        //Pin de entrada del encoder 2 asociado al motor 2
#define pin_encoder3  39        //Pin de entrada del encoder 3 asociado al motor 3

#define pin_IF_LL     36        //Pin de entrada del sensor infrarrojo Izquierdo-Izquierdo
#define pin_IF_L      34        //Pin de entrada del sensor infrarrojo Izquierdo
#define pin_IF_C      33        //Pin de entrada del sensor infrarrojo Central
#define pin_IF_R      14        //Pin de entrada del sensor infrarrojo Derecho
#define pin_IF_RR     15        //Pin de entrada del sensor infrarrojo Derecho-Derecho
   
#define SPEED         180
#define ROT_SPEED     120
#define BIG_STOP_TIME 1000
#define STOP_TIME     1000
#define LITLE_STOP_TIME 250
#define SKIP_LINE     200
#define ROT_90G       27
#define GO_HALF_CAR   8 


//const uint8_t channels[4][2]={{0,1},{2,3},{4,5},{6,7}};
//static int8_t pin_to_channel[SOC_GPIO_PIN_COUNT] = { 0 };
//const uint8_t IF[]={pin_IF_LL,pin_IF_L,pin_IF_C,pin_IF_R,pin_IF_RR};
//volatile uint8_t runningm;


typedef struct{
  int32_t     cnt;          //Almacena el cuenteo actual ya sea hacia delante o hacia atras
  int32_t     max_count;    //Numero de pasos que contará antes de hacer cualquier operación
  int8_t      dir;          //Dirección en la que contará los pasos hacia adelante o hacia atras
  uint8_t     auto_stop;    //auto_stop == 1 irq subrutina stop motor after n count of encoder
  uint8_t     stop_request; //stop_request set to 1 when cnt reached to max_count; 
}encoders_t;


encoders_t encoders[4]={0};
const uint8_t motors[4][2]={{pin_motor0A,pin_motor0B},{pin_motor1A,pin_motor1B},{pin_motor2A,pin_motor2B},{pin_motor3A,pin_motor3B}};
uint8_t sensor;
TaskHandle_t Task1;

//Interrupt subroutine para el encoder0           
void IRAM_ATTR encoder0() {
  encoders[0].cnt+=encoders[0].dir;
  if (abs(encoders[0].cnt)>=abs(encoders[0].max_count)) {
    encoders[0].stop_request=1;
    if (encoders[0].auto_stop){
      analogWrite(motors[0][0],0);
      digitalWrite(motors[0][1],LOW);
    }
  }
}

//Interrupt subroutine para el encoder1
void IRAM_ATTR encoder1() { 
  encoders[1].cnt+=encoders[1].dir;
  if (abs(encoders[1].cnt)>=abs(encoders[1].max_count)) {
    encoders[1].stop_request=1;
    if (encoders[1].auto_stop){
      analogWrite(motors[1][0],0);
      digitalWrite(motors[1][1],LOW);
    }
  }
}

//Interrupt subroutine para el encoder2
void IRAM_ATTR encoder2() { 
  encoders[2].cnt+=encoders[2].dir;
  if (abs(encoders[2].cnt)>=abs(encoders[2].max_count)) {
    encoders[2].stop_request=1;
    if (encoders[2].auto_stop){
      analogWrite(motors[2][0],0);
      digitalWrite(motors[2][1],LOW);
    }
  }
}

//Interrupt subroutine para el encoder3
void IRAM_ATTR encoder3() { 
  encoders[3].cnt+=encoders[3].dir;
  if (abs(encoders[3].cnt)>=abs(encoders[3].max_count)) {
    encoders[3].stop_request=1;
    if (encoders[3].auto_stop){
        analogWrite(motors[3][0],0);
        digitalWrite(motors[3][1],LOW);
    }
  }
}

//Establece la dirección o frenado un motor
//dir puede ser: FORWARD,BACKWAR,BRAKE0,BRAKE1
//FORWARD:  El motor gira hacia adelante
//BACKWARD: El motor gira hacia atras
//BRAKE0, BRAKE1: EL motor se para 
void set_motor(const uint8_t motor[], uint8_t dir){ 
  analogWrite(motor[0], (dir & 1) ? HIGH:LOW);
  digitalWrite(motor[1], (dir & 2) ? HIGH:LOW);
}

//Establece la velocidad para las parejas de motores Izquierdos y Derechos
//Lspeed: Puede ser positivo (hacia adelante) o negativo (hacia atras)
//Rspeed: Puede ser positivo (hacia adelante) o negativo (hacia atras)
void set_speed(const uint8_t motor[][2], int16_t Lspeed, int16_t Rspeed){
  uint8_t Ls = (abs(Lspeed) > 255) ? 255 : abs(Lspeed);
  uint8_t Rs = (abs(Rspeed) > 255) ? 255 : abs(Rspeed);
  
  analogWrite(motor[0][0], ( (Lspeed<0) ? 255-Ls : Ls)); 
  digitalWrite(motor[0][1], ( (Lspeed<0) ? HIGH : LOW));

  analogWrite(motor[1][0], ( (Rspeed<0) ? 255-Rs : Rs)); 
  Serial.printf("Motor 1A: %d \n\r",(Rspeed<0) ? 255-Rs : Rs);
  digitalWrite(motor[1][1], ( (Rspeed<0) ? HIGH : LOW));

  analogWrite(motor[2][0], ( (Lspeed<0) ? 255-Ls : Ls)); 
  digitalWrite(motor[2][1], ( (Lspeed<0) ? HIGH : LOW));

  analogWrite(motor[3][0], ( (Rspeed<0) ? 255-Rs : Rs)); 
  digitalWrite(motor[3][1], ( (Rspeed<0) ? HIGH : LOW));
}

//Esta funcion establece los valores de las estructuras encoders
//donde se indica para cada motor el valor de pasos que debe andar (enx)
//Se resetea el valor de pasos andados actualmente
//Se establece la dirección y si el motor debe pararse tras alcanzar el numero de pasos solicitado
//en0, en1, en2, en3: numero de pasos a contar hacia adelante o hacia atras segun el signo para cada motor
//stop_bits:Cada bit indica si el respectivo motor debe pararse tras alcanzar el numero de pasos solicitado (enx)
void set_encoders(int32_t en0, int32_t en1, int32_t en2, int32_t en3, uint8_t stop_bits){
encoders[0].cnt=0;
encoders[0].max_count=en0;
encoders[0].dir=(en0<0) ? -1:1;
encoders[0].auto_stop=(stop_bits & 1);
encoders[0].stop_request=0;

encoders[1].cnt=0;
encoders[1].max_count=en1;
encoders[1].dir=(en1<0) ? -1:1;
encoders[1].auto_stop=(stop_bits & 2)>>1;
encoders[1].stop_request=0;

encoders[2].cnt=0;
encoders[2].max_count=en2;
encoders[2].dir=(en2<0) ? -1:1;
encoders[2].auto_stop=(stop_bits & 4)>>2;
encoders[2].stop_request=0;

encoders[3].cnt=0;
encoders[3].max_count=en3;
encoders[3].dir=(en3<0) ? -1:1;
encoders[3].auto_stop=(stop_bits & 8)>>3;
encoders[3].stop_request=0;
}

//Lectura de los sensores de infrarrojos
//Devuelve un valor de 5 bits donde cada uno representa el estado de cada uno de los sensores de infrarojos
uint8_t read_sensor_values() { 
  uint8_t sensor; 
  sensor=(((uint8_t)digitalRead(pin_IF_LL) & 0x01) << 4) | (((uint8_t)digitalRead(pin_IF_L) & 0x01) << 3) | 
         (((uint8_t)digitalRead(pin_IF_C) & 0x01) << 2) | (((uint8_t)digitalRead(pin_IF_R) & 0x01) << 1) | 
         (((uint8_t)digitalRead(pin_IF_RR) & 0x01));
  return sensor;
}

//Chequea si hay una linea negra entre los sensores
//Saliendo con true en caso contrario sale con false
//Si alcanza una linea perpendicular para los motores
//Devolviendo el valor false
//Esto permite seguir la linea mientras el valor sea
//true (while(tracking_blackline()); saliendo de este
//bucle cuando cuando todos los sensores estan off
uint8_t tracking_blackline(){
  static uint8_t previous_sensor=0;   
  uint8_t freturn=1;
  //sensor= (~read_sensor_values()) & 0b11111;
  sensor= read_sensor_values();

  if (sensor==0b10000 ) { 
      set_speed(motors,-SPEED,SPEED);
    }
  if (sensor==0b11000 || sensor==0b01000) { 
      set_speed(motors,SPEED/2,SPEED);
    }
  if (sensor==0b00100 || sensor==0b01100 || sensor==0b00110) {  
      set_speed(motors,SPEED,SPEED);
    }    
  if (sensor==0b00011 || sensor==0b00010) {  
      set_speed(motors,SPEED,SPEED/2);
    }
    if (sensor==0b00001 ) {  
      set_speed(motors,SPEED,-SPEED);
    }
    if ((sensor&0b01110) == 0b01110 ){
      previous_sensor=sensor;
      Serial.println("Saved sensor"); 
      set_speed(motors,0,0);
    }
      
    if (sensor==0b11111) {// && ((previous_sensor&0b01110)==0b01110)) {
      Serial.println("Stopped"); 
      set_speed(motors,0,0);
      delay(LITLE_STOP_TIME);
      previous_sensor=0;
      freturn=0;
    }  
    return freturn;  
}

//Avance de N pasos hacia adelante
//enc:   Numero de pasos a avanzar
//Speed: Velocidad a la que avanzará
void go_cnt(int32_t enc, uint8_t Speed){
  set_encoders(enc,enc,enc,enc,0b1111);
  set_speed(motors,Speed, Speed);
  Serial.println("go_cnt");
  while(!(encoders[0].stop_request && encoders[1].stop_request && encoders[2].stop_request && encoders[3].stop_request))
    delay(10);
  Serial.println("End_cnt");
}

//Rotación de N pasos del enconder hacia la Izquierda
void Lrotate_cnt(int32_t enc, uint8_t Speed){
  enc=abs(enc);
  set_encoders(-enc,enc,-enc,enc,0b1111);
  set_speed(motors,-Speed, Speed);
  Serial.println("Rotamos a la izquierda");
  while(!(encoders[0].stop_request && encoders[1].stop_request && encoders[2].stop_request && encoders[3].stop_request));
}

//Rotación de N pasos del enconder hacia la derecha
void Rrotate_cnt(int32_t enc, uint8_t Speed){
  enc=abs(enc);
  set_encoders(enc,-enc,enc,-enc,0b1111);
  set_speed(motors,Speed, -Speed);
  while(!(encoders[0].stop_request && encoders[1].stop_request && encoders[2].stop_request && encoders[3].stop_request));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Setup 1 Init");
  
  pinMode(pin_encoder0,INPUT);
  pinMode(pin_encoder1,INPUT);
  pinMode(pin_encoder2,INPUT);
  pinMode(pin_encoder3,INPUT);
  
  pinMode(pin_IF_LL,INPUT);
  pinMode(pin_IF_L,INPUT);
  pinMode(pin_IF_C,INPUT);
  pinMode(pin_IF_R,INPUT);
  pinMode(pin_IF_RR,INPUT);
  
  
  for (uint8_t i=0; i<4; i++){
    pinMode(motors[i][0],OUTPUT); 
    pinMode(motors[i][1],OUTPUT);
  }
  
  set_motor(motors[0],BRAKE0);
  set_motor(motors[1],BRAKE0);
  set_motor(motors[2],BRAKE0);
  set_motor(motors[3],BRAKE0);
  
  delay(2000);
  
  attachInterrupt(pin_encoder0, encoder0, CHANGE);
  attachInterrupt(pin_encoder1, encoder1, CHANGE);
  attachInterrupt(pin_encoder2, encoder2, CHANGE);
  attachInterrupt(pin_encoder3, encoder3, CHANGE);
  
// Create new Process on other core
  xTaskCreatePinnedToCore(
    setup2, /* Function to implement the task */
    "Task1", /* Name of the task */
    30000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */

  Serial.println("Setup 1 finished");
}

// Loop executed on core 1
void loop() {
  uint8_t e0, e1, e2, e3;
  
  set_encoders(0,0,0,0,0);
  while(tracking_blackline());
  Serial.println("Hemos llegado");
  delay(BIG_STOP_TIME);
  go_cnt(GO_HALF_CAR,SPEED);  
  delay(LITLE_STOP_TIME);
  Lrotate_cnt(ROT_90G,ROT_SPEED);
  //Lrotate_tosensor(0b11100,ROT_SPEED);
  delay(LITLE_STOP_TIME);
  
  set_encoders(0,0,0,0,0);
  while(tracking_blackline());
  Serial.println("Hemos llegado al nodo 2");
  delay(BIG_STOP_TIME);
  go_cnt(GO_HALF_CAR,SPEED);
  delay(LITLE_STOP_TIME);
  Lrotate_cnt(ROT_90G,ROT_SPEED);
  delay(LITLE_STOP_TIME);

  set_encoders(0,0,0,0,0);
  while(tracking_blackline());
  Serial.println("Hemos llegado al nodo 3");
  delay(BIG_STOP_TIME);
  go_cnt(GO_HALF_CAR,SPEED);
  delay(LITLE_STOP_TIME);
  Lrotate_cnt(ROT_90G,ROT_SPEED);
  delay(LITLE_STOP_TIME);

  set_encoders(0,0,0,0,0);
  while(tracking_blackline());
  Serial.println("Hemos llegado al nodo 3");
  delay(BIG_STOP_TIME);
  go_cnt(GO_HALF_CAR,SPEED);
  delay(LITLE_STOP_TIME);
  Lrotate_cnt(ROT_90G,ROT_SPEED);
  delay(LITLE_STOP_TIME);
  
  set_encoders(0,0,0,0,0);
  while(tracking_blackline());
  Serial.println("Destino final");
  
  while(1);
    
}

//Global Var declaration para core 0
char str16b[12];
uint16_t* rawData=(uint16_t*)malloc(2048);   //Full ram memory of attiny
char* str_graph=0;
char hexDatalen[5]={0};
uint16_t dataLen=0;
uint16_t dataRest=0;
uint16_t dataReaded=0;

void setup2(void * parameter) {

  Serial2.begin(115200);

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Create SoftAP
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);


  Serial.print("Access point available to connect to: ");
  Serial.println(ssid);

 // server.on("/", handle_root);

    // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");});
    

  /* PROBLEMA: Esta función solo puede ponerse aquí en el SETUP porque en el loop 
   *  se queda pillada. Pero esto en el setup solo se ejecuta una vez 
   *  En ejemplo se llamaba a una función del sensor que leía el valor y lo convertía
   *  a string pero igualmente estaba en el setup, ¿Como salen valores distintos?
   *  De todos modos si pudiera mandar en el mismo string todos los datos...
   */
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", str_graph);
  });

  server.begin();
  Serial.println("HTTP server started");
  delay(100);

  while(1) loop2(); // As we don´t have auto loop ejecution we call loop2 infintely
}

void loop2() {
  uint8_t cget;
  uint32_t tstart=micros();
  Serial.println("Loop 2 init...");
  while(1) {
    do {
      while(Serial2.available()) {
        cget=Serial2.read();
        tstart=micros();
      }
    } while ( micros() < tstart+5000);
    Serial.println("Time out detected...");
    
    while (Serial2.available()<7);
    Serial.println("Seven bytes receibed...");
    while (!Serial2.find("Init:0X",7));
    Serial.println("Header Init:0X detected...");
    while (Serial2.available()<4);
  
    Serial2.readBytes(hexDatalen,4);
    dataLen= (hexDatalen[0]-'0')<<12 | (hexDatalen[1]-'0')<<8 | (hexDatalen[2]-'0')<<4 | (hexDatalen[3]-'0'); 
    Serial.printf("Reading dataLen: %s= %d\n\r",hexDatalen,dataLen);
    memset(rawData,0,dataLen+1);
    dataRest=dataLen;
    dataReaded=0;
    while (dataRest) {
      dataReaded=Serial2.readBytes((uint8_t*)(rawData+dataReaded),dataRest);
      dataRest-=dataReaded;
    }
    Serial.println("Buffer readed susefull...");
  
    if(!str_graph)
      str_graph=(char*)malloc(dataLen*11+1);
      
    memset(str_graph,0x0,dataLen*11+1);  
    for (uint16_t i=0;i<dataLen>>1;i++) {
      sprintf(str16b,"%d,",*(rawData+i));
      strcat(str_graph,str16b);
    }
  }  
}
