float t_anterior = 0;
float dt_anterior = 0;
float tempC;
int reading;
int tempPin = 0;
float error=0;
float tref=-2.6;
float kp=11;
float ki=44;
float kd=-0.01;
float erroracum=0;
float errorant=0;
int pid,p,i,d,u=0;
float salida;
int STBY = 10;        // definimos el pin de standby
int PWMA = 3;        // control de velocidad
int AIN1 = 9;                           // dirección
int AIN2 = 8;                           // dirección
// para el motor B
int PWMB = 5;        // control de velocidad
int BIN1 = 11;                         // dirección
int BIN2 = 12;    
byte outputValue = 0;        // valor del PWM
     // valor del PWM

////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0
#define G_R 131.0
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
float Acc[2];
float Gy[2];
float Angle[2];

void mover(int motor, int velocidad, int direccion){    //Vamos a definir la función mover, que va                                                                                            a acciona un motor, fijar su velocidad y el                                                                                            sentido de giro. Definimos:    //motor:                                                                                                    llamaremos 1 al motor A, y 2 al motor B
                                          //velocidad: desde 0 a 255
       //dirección: 0 para el giro en sentido horario, 1 sentido                                  antihorario
digitalWrite(STBY, HIGH);          //deshabilitar standby para mover
 boolean inPin1 = HIGH;             // creamos la variable booleana (solo puede ser HIGH/LOW) inpin1
 boolean inPin2 = LOW;             // y le asignamos el valor LOW. A inPin2 le damos el valor                                                                HIGH
if(direccion == 1){
inPin1 = LOW;
inPin2 = HIGH;
}
if(motor == 1){
digitalWrite(AIN1, inPin1);
digitalWrite(AIN2, inPin2);
 analogWrite(PWMA, velocidad);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, velocidad);
  }
}
void setup()
{

  //////////////////////////////////////////////////////////////////////77
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
  ////////////////////////////////////////////////////////////////////////7
//analogReference(INTERNAL);
Serial.begin(9600);
 pinMode(STBY, OUTPUT);  // definimos esos pines como salidas
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
Serial.print("Angulo");
Serial.print(" PID");

erroracum = 0;
errorant=0;
}

void loop()
{

  //////////////////////////////////////////////////////////////777777
//Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
    //Se calculan los angulos Y, X respectivamente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*0.010) + 0.02*Acc[0];
  
 Angle[1] = 0.98 *(Angle[1]+Gy[1]*0.010) + 0.02*Acc[1];
 Serial.print("");
 Serial.println(Angle[1]);
tempC=(Angle[1]+3);
float t = millis();
float Tiempo_iter = t - t_anterior;
if (Tiempo_iter >= 3){
    float dt = millis();
    float deltaT= dt - dt_anterior;
    dt_anterior = dt;
    error=tref-tempC;
    erroracum =erroracum + error*deltaT/650;
    p=(kp*error);
    i=(ki*erroracum);
    d=kd*(errorant-error)/deltaT*650; 
    pid =p+i+d;
    u=abs(pid);
    errorant=error;
    t_anterior = t;
if(tempC>tref){
mover(1, u+7, 1);                                // acciona el motor 1, velocidad máxima, adelante
mover(2, u, 1); 
}
   else{
  mover(1, u+7, 0);                                // acciona el motor 1, velocidad media , atrás
  mover(2, u, 0); 
     }
}
}

