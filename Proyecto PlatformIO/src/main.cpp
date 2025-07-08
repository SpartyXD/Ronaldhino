#include <Arduino.h>
#include <DualVNH5019MotorShield.h>

//=====================================
//MACROS / PARAMETERS / AUX FUNCTIONS
#define rep(i, n) for(int i=0; i<n; i++)
#define MAX_ARDUINO_TIME 3294967295

unsigned long get_time(){
    return (millis()%MAX_ARDUINO_TIME);
}

//=======================================
// Definición de PINS
#define encoder0PinA  19
#define encoder0PinB  18
#define encoder1PinA  20
#define encoder1PinB  21

//CONSTANTES
float RPM = 31250;

// Variables Tiempo
unsigned long time_ant = 0;
const int Period = 10000;   // 10 ms = 100Hz
const float dt = Period *0.000001f;

// Variables de los Encoders y posicion
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;
unsigned long newtime;
float vel0;
float vel1;

//-----------------------------------
// CONFIGURANDO INTERRUPCIONES
void doEncoder0A()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder0B()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

void doEncoder1A()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder1B()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}


void initEncoders(){
    // Configurar Encoders
    pinMode(encoder0PinA, INPUT);
    digitalWrite(encoder0PinA, HIGH);       // Incluir una resistencia de pullup en le entrada
    pinMode(encoder0PinB, INPUT);
    digitalWrite(encoder0PinB, HIGH);       // Incluir una resistencia de pullup en le entrada
    pinMode(encoder1PinA, INPUT);
    digitalWrite(encoder1PinA, HIGH);       // Incluir una resistencia de pullup en le entrada
    pinMode(encoder1PinB, INPUT);
    digitalWrite(encoder1PinB, HIGH);       // Incluir una resistencia de pullup en le entrada
    attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);  // encoder 0 PIN A
    attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);  // encoder 0 PIN B
    attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  // encoder 1 PIN A
    attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);  // encoder 1 PIN B
}


void update_info(){
  newposition0 = encoder0Pos;
  newposition1 = encoder1Pos;

  //-----------------------------------
  // Calculando Velocidad del motor en unidades de RPM
  vel0 = -(float)(newposition0 - oldposition0) * RPM / (newtime - time_ant); //RPM
  vel1 = -(float)(newposition1 - oldposition1) * RPM / (newtime - time_ant); //RPM
  oldposition0 = newposition0;
  oldposition1 = newposition1;
}


//=================================
//Objetos globales

struct customPID{
    //CONSTANTES
    float Kp = 0;
    float Ki = 0;
    float Kd = 0; 

    int MAX_PID_VAL = 0;

    // Variables PID
    float P, I, D, PID_VAL;
    float error = 0;
    float last_error = 0;
    float sum = 0;

    //Inicializar PID
    void init(float kp, float ki, float kd, int max_pid){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        MAX_PID_VAL = max_pid;

        //Inicializar errores
        reset_error();
    }


    int PID(int actual, int desired){
        //Calcular error
        error = desired - actual;
        sum = constrain(sum+error, -MAX_PID_VAL, MAX_PID_VAL); 

        //Calcular PID
        P = error;
        I = sum; 
        D = error - last_error;

        //Calcular salida PID
        PID_VAL =  Kp*P + Kd*D + Ki*I; 
        PID_VAL = constrain(PID_VAL, -MAX_PID_VAL, MAX_PID_VAL);

        last_error = error;
        return PID_VAL;
    }

    void reset_error(){
        error = 0;
        last_error = 0;
        sum = 0;
    }

};

DualVNH5019MotorShield md;
customPID speedPIDLeft, speedPIDRight;

//=================================

//-----------------------------------
//CONSTANTES / FUNCIONES AUX
#define MAX_OUTPUT 200
#define BASE_SPEED 150

//-----------------------------------

char endChar = ';'; // Carácter que indica el final del mensaje
String incomingMessage; // Variable donde se almacenará el mensaje recibido
int left_speed=0, right_speed=0; //Velocidades

//=================================
//Funciones globales
void handleCommand(String &command);

//=================================

void setup(){
  Serial.begin(115200);
  Serial.println("Iniciando ronaldhino"); // Monitor serial
  Serial3.begin(38400); // Comunicación Bluetooth

  md.init();
  initEncoders();
  speedPIDLeft.init(10, 0.3, 1, MAX_OUTPUT);
  speedPIDRight.init(7, 0.3, 1, MAX_OUTPUT);
}


void loop(){
  // Verificamos si hay datos disponibles desde Bluetooth
  if (Serial3.available() > 0){
    incomingMessage = Serial3.readStringUntil(endChar);
    Serial.println("Comando recibido: " + incomingMessage);
    handleCommand(incomingMessage);
  }

  //Esperar tiempo muestreo
  if(micros() - time_ant < Period)
    return; // Esperar el tiempo de muestreo

  newtime = micros();

  //Actualizar encoders
  update_info();

  //PID velocidad motor
  int left_output = speedPIDLeft.PID(vel0, left_speed);
  int right_output = speedPIDRight.PID(-vel1, right_speed);

  //Actualizar motores
  md.setM1Speed(-left_output); // Motor izquierdo
  md.setM2Speed(right_output); // Motor derecho

  time_ant = newtime;
}


void handleCommand(String &command){
    char letter = command[0];
    int n = command.length();

    if(letter == 'S'){
      left_speed=right_speed=0;
    }
    else if(letter == 'V'){
      int separator = command.indexOf(' ');
      left_speed = command.substring(1, separator).toInt();
      right_speed = command.substring(separator+1, n).toInt();

      left_speed = constrain(left_speed, -BASE_SPEED, BASE_SPEED);
      right_speed = constrain(right_speed, -BASE_SPEED, BASE_SPEED);

      Serial.println("Nuevas velociades: " + String(left_speed) + " " + String(right_speed));
    }

}