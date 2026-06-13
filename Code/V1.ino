#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// --- CONFIGURACIÓN DE PINES ---
const int IN1 = 3;  
const int IN2 = 5;  
const int IN3 = 10; 
const int IN4 = 11; 

float Kp = 400.0;  
float Ki = 40.0;   
float Kd = 0.5;   

// --- VARIABLES DE CONTROL Y CALIBRACIÓN ---
float setpoint = 0.0; 
float anguloOffset = 0.0; 

float error, lastError, proporcional, integral, derivativo, controlSalida;
unsigned long lastTime;
double dt;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  detenerMotores();

  Serial.println("Buscando IMU MPU6050...");
  byte status = mpu.begin();
  
  if(status != 0){ 
    Serial.println("ERROR: No se encuentra el IMU.");
    while(1){ } 
  } 
  
  // Tiempo extendido para estabilizar la posición física
  Serial.println(">>> COLOCA EL PÉNDULO EN SU POSICIÓN DE EQUILIBRIO PERFECTO <<<");
  Serial.println("Mantenlo totalmente quieto. Calculando ceros en 4 segundos...");
  delay(4000); 
  
  Serial.println("CalibrandoOffsets internos del chip...");
  mpu.calcOffsets(true); 
  delay(500);

  // --- FILTRO CORRECTOR DE DESFASE (Aquí eliminamos esos ~2 grados sobrantes) ---
  Serial.println("Eliminando error residual de inclinación...");
  float sumaAngulo = 0;
  int lecturas = 150; // Aumentamos a 150 lecturas para mayor precisión
  
  for(int i = 0; i < lecturas; i++) {
    mpu.update();
    sumaAngulo += mpu.getAngleY();
    delay(10);
  }
  
  // Guardamos el promedio exacto (por ejemplo esos -2.04) para restarlo siempre
  anguloOffset = sumaAngulo / (float)lecturas; 
  
  Serial.print("¡Calibración exitosa! Tu desfase real era de: ");
  Serial.print(anguloOffset);
  Serial.println("°. Aplicando corrección por software...");
  delay(1500);
  
  lastTime = millis();
}

void loop() {
  mpu.update();
  
  unsigned long currentTime = millis();
  dt = (double)(currentTime - lastTime) / 1000.0;
  if (dt <= 0.0) dt = 0.01; 
  lastTime = currentTime;

  // CORRECCIÓN MATEMÁTICA: Restamos el offset calculado automáticamente
  float anguloActual = mpu.getAngleY() - anguloOffset; 

  // --- CÁLCULO DEL PID ---
  error = setpoint - anguloActual; 
  proporcional = error * Kp;
  integral += error * Ki * dt;
  integral = constrain(integral, -150, 150); 
  derivativo = ((error - lastError) / dt) * Kd;
  lastError = error;

  controlSalida = proporcional + integral + derivativo;

  // Seguridad: Si se cae a más de 35 grados, apagar motores
  if (abs(anguloActual) > 35.0) {
    detenerMotores();
    integral = 0; 
  } else {
    controlarMotores(controlSalida);
  }

  // --- REVISAR DATOS EN EL MONITOR ---
  Serial.print("Ángulo_Actual: "); 
  Serial.print(anguloActual); // Ahora verás que oscila muy cerca de 0.00
  Serial.print(" | Fuerza_Calculada: "); 
  Serial.println(controlSalida);
  
  delay(15); 
}

void controlarMotores(float salida) {
  int velocidad = constrain(abs(salida), 0, 255);

  if (salida > 0) { 
    analogWrite(IN1, velocidad);
    analogWrite(IN2, 0);
    analogWrite(IN3, velocidad);
    analogWrite(IN4, 0);
  } else if (salida < 0) { 
    analogWrite(IN1, 0);
    analogWrite(IN2, velocidad);
    analogWrite(IN3, 0);
    analogWrite(IN4, velocidad);
  } else {
    detenerMotores();
  }
}

void detenerMotores() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}