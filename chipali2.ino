// Seguidor de Línea PID "CHIPALI"
// Daniel Timothy Krekler Caudillo
// danielkrekler@gmail.com
// Ing Mecatrónica, Universidad de Sonora.

#include <QTRSensors.h>

#define KP               0.05
#define KD               0
#define KI               0

#define M1_DEFAULT_SPEED    80
#define M2_DEFAULT_SPEED    80
#define M1_MAX_SPEED  180
#define M2_MAX_SPEED  180

#define NUM_SENSORS    8                     //Numero de sensores utilizados
#define TIMEOUT        2500                  //Espera 2500 microsegundos para que los sensores vayan a LOW
#define EMITTER_PIN    12                    //Utilizado para calibrar. Es el encargado de prender los IR.
#define DEBUG          0                     //Por si se ocupa la salida serial debug.

//Crea objeto para 8 sensores QTR-8RC, en los pins:
QTRSensorsRC qtrrc((unsigned char[]) {
  2, 3, 4, 5, 6, 7, 8, 9
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

//boolean arranque = true;

int pwm1 = 10; //left
int pwm2 = 11; //right EN1

int m1A = A2; //right
int m1B = A3; //right

int m2A = A4; //left
int m2B = A5; //left

int lastError = 0;
int lastProportional = 0;
int integral = 0;


//Control de Motores
void set_motors(int motor1speed, int motor2speed)
{
  // MOTOR 1
  //Si se pasan de velocidad, poner velocidad maxima.
  if (motor1speed > M1_MAX_SPEED)
    motor1speed = M1_MAX_SPEED;
  // Si velocidad es mayor que 0, ir para enfrente
  if (motor1speed > 0) {
    digitalWrite(m1A, LOW);
    digitalWrite(m1B, HIGH);

    //analogWrite(pwm1, motor1speed);
  }
  //Si hay velocidad negativa, ir para atras.
  else if (motor1speed < 0) {
    digitalWrite(m1A, HIGH);
    digitalWrite(m1B, LOW);

    motor1speed = -motor1speed;
    //analogWrite(pwm1, -motor1speed);
  }
  // MOTOR 2
  if (motor2speed > M2_MAX_SPEED)
    motor2speed = M2_MAX_SPEED;
  if (motor2speed > 0) {
    digitalWrite(m2A, LOW);
    digitalWrite(m2B, HIGH);

    //analogWrite(pwm2, motor2speed);
  }
  else if (motor2speed < 0) {
    digitalWrite(m2A, HIGH);
    digitalWrite(m2B, LOW);

    motor2speed = -motor2speed;

    //analogWrite(pwm2, -motor2speed);
  }
  //Correr Motores!
  analogWrite(pwm1, motor1speed);
  analogWrite(pwm2, motor2speed);
}

//calibrar sensores
void calibration()
{
  // pinMode(A2, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // Prender led de Arduino
  for (int i = 0; i < 100; i++)  // Calibracion. Dura unos segundos
  {

    qtrrc.calibrate(QTR_EMITTERS_ON);       // Lee todos los sensores (i.e. ~25 ms per call)
    delay(20);
  }
  digitalWrite(13, LOW);     // Apagar led de Arduino


  //Por si se ocupa
  /*if (DEBUG)
    {
    // Imprime valores minimos de la calibracion, cuando los leds estaban prendidos.
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
     Serial.print(qtrrc.calibratedMinimumOn[i]);
     Serial.print(' ');
    }
    Serial.println();


    // Imprime valores maximos de la calibracion, cuando los leds estaban prendidos.
    for (int i = 0; i < NUM_SENSORS; i++)
    {
     Serial.print(qtrrc.calibratedMaximumOn[i]);
     Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    }*/
}

void setup()
{
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);

  pinMode(m1A, OUTPUT);
  pinMode(m1B, OUTPUT);

  pinMode(m2A, OUTPUT);
  pinMode(m2B, OUTPUT);

  calibration();

  set_motors(0, 0);

  //while(digitalRead(A2) != HIGH);
}



void loop()
{
  
/*if(arranque){
    //set_motors(255, 255);
    }
  arranque = false
*/
  int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 1);     //Lee la Linea blanca.
  //  int position = qtrrc.readLine(sensorValues); // NEGRA!!

  
  int error = position - 3500;

  int turn = (KP * error) + (KI * integral) + (KD * (error - lastError));
  integral += error;
  lastError = error;
  //if (integral>1000) integral=1000; //limitamos la integral para no causar problemas
  //if (integral<-1000) integral=-1000;

  int leftMotorSpeed = M1_DEFAULT_SPEED - turn;
  int rightMotorSpeed = M2_DEFAULT_SPEED + turn;

  set_motors(leftMotorSpeed, rightMotorSpeed);
}
