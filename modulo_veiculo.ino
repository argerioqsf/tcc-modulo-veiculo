#include "Wire.h"
#include <Sim800l.h>
#include <SoftwareSerial.h>
Sim800l Sim800l;

#define MPU_ADDR  0x68

#define STOP_OFFSET  300

uint8_t mpu6050_init();
void mpu6050_offset(int16_t cal_ac_x, int16_t cal_ac_y, int16_t cal_ac_z);
void mpu6050_write(uint8_t address, int16_t val, uint8_t size);
int16_t mpu6050_read(uint8_t address, uint8_t size);

bool error;
String textSms, numberSms;

unsigned long t_amostra = 0;
unsigned long t_parado = 0;
bool parado = true;
float dist = 0;
float forca_g = 0;
float acel2 = 0;
float vel2 = 0;
void setup()
{
  Serial.begin(9600);
  Sim800l.begin();
  
  Serial.print("Limpando SMS antigos...");
  error = Sim800l.delAllSms(); //Apaga SMS
  Serial.println(" Apagados!");

  uint8_t resp = mpu6050_init();

  if (!resp)
  {
    Serial.println("Falha na conexão do MPU6050");
  }
  else
  {
    Serial.println("MPU6050 conectado");
  }

  // Calibração
  // Comente esta linha se você não sabe os valores do seu sensor
  //mpu6050_offset(-2267, -1111, 504);

  t_amostra = micros();
}

void loop()
{
  int16_t ac_y = mpu6050_read(0x3D, 2);
  float aux_ac = float(ac_y);
  
  if(fabs(aux_ac) < STOP_OFFSET && abs(millis() - t_parado) > 50)
  {
    parado = true;
   
    t_amostra = micros();
  }
  
  else if(fabs(aux_ac) >= STOP_OFFSET)
  {
    t_parado = millis();
    parado = false;
  }
  aux_ac = ((aux_ac + 32768.0) * 4.0/65536.0 - 2.0) * 9.81;

  if (parado)
  {
     if(forca_g >= 10.00){
        Serial.println("forca G (em y) 2: ");
        Serial.println(forca_g);
        Serial.println("Enviando para a central...");
        int str_len = numberSms.length() + 1;
        char numberSmsArray[str_len];
        numberSms.toCharArray(numberSmsArray, str_len);
        String forca_g_string = String(forca_g,2);
        String stringInfo = "lat:-0.0363468,lon:-51.1728199,forca:"+(String)forca_g_string;
        int stringInfo_len = stringInfo.length() + 1;
        char stringInfo_array[stringInfo_len];
        stringInfo.toCharArray(stringInfo_array,stringInfo_len);
        Serial.println("stringInfo:");
        Serial.println(stringInfo_array);
        error=Sim800l.sendSms("+5596981167329",stringInfo_array);
        //numero damiles Claro +5596984223832
        //numero damiles Tim +5596981167329
        Serial.println("SMS enviado para a central.");
     }
    dist = 0.0;
    forca_g = 0.0;
    acel2 = 0.0;
    vel2 = 0.0;
  }
  // Está se movendo
  else
  {
    t_amostra = micros() - t_amostra;
 
    dist = calculo_trapezio(dist, aux_ac, t_amostra, 1);
    forca_g = calculo_trapezio(dist, aux_ac, t_amostra, 2);
   
    t_amostra = micros();
  }
}

float calculo_trapezio(float dist, float acel, unsigned long tempo, int tipo)
{
  static float last_acel = 0.0;
  static float last_vel = 0.0; 
  float vel;
  float t = (float)tempo/1000000.0;
  if(dist == 0.0)
  {
    last_vel = 0.0;
    last_acel = 0.0;
  }
  vel = last_vel + (last_acel + acel) * t / 2.0;
  

  dist = dist + (last_vel + vel) * t / 2.0;
  vel2 = dist/t;
  acel2 = vel2 * dist;
  forca_g = acel2 / 9.81;
  last_acel = acel;
  last_vel = vel;
  if(tipo == 1){
    return dist;
  }
  if(tipo == 2){
    return forca_g;
  }
}

uint8_t mpu6050_init()
{
  Wire.begin();
  uint8_t id = mpu6050_read(0x75, 1);
  if(id != MPU_ADDR)
  {
    return 0;
  }
  mpu6050_write(0x6B, 4, 1);
  return 1;
}

void mpu6050_offset(int16_t cal_ac_x, int16_t cal_ac_y, int16_t cal_ac_z)
{
  // Acel x
  mpu6050_write(0x06, cal_ac_x, 2);

  // Acel y
  mpu6050_write(0x08, cal_ac_y, 2);
 
  // Acel z
  mpu6050_write(0x0A, cal_ac_z, 2);
}

void mpu6050_write(uint8_t address, int16_t val, uint8_t size)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(address);
  if(size == 2)
  {
    Wire.write(val >> 8); // MSByte
  }
  Wire.write(val & 0xFF); // LSByte
  Wire.endTransmission();
}

int16_t mpu6050_read(uint8_t address, uint8_t size)
{
  int16_t data = 0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(address);
  Wire.endTransmission(false);
 
  Wire.requestFrom(MPU_ADDR, size);
  if(size == 2)
  {
    data = Wire.read() << 8;
  }
  data |= Wire.read();

  return data;
}
