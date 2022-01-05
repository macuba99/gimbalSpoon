// Copyright (C) 2022 lbins_gr1 and contributors.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#define BNO055_SAMPLERATE_DELAY_MS (10)           // Periodo de muestreo de los datos de orientación absoluta.
#define LM35 A1                                   // Asignar la entrada del sensor de temperatura al pin analógico A1

Adafruit_BNO055 bno = Adafruit_BNO055(1,0x29);    // Crear una instancia del sensor y asignarle una dirección

Servo Servo1;   
Servo Servo2;   
Servo Servo3;                                     

double roll, pitch, heading;                      // Ángulos de euler

float heading_referencia = 0.0;                   // Orientación deseada (heading)
float diferencia = 0.0;                           // Diferencia entre la orientación actual y la deseada
float umbral_heading = 24.0;                      // Desviación máxima con respecto a heading_referencia
float n_iter_heading = 15.0;                      // Número de iteraciones en las que se recalcula la referencia
int contador = n_iter_heading+1;                  // Indica si se ha terminado de recalcular la referencia
float paso = 0.0;                                 // Cantidad que se le añade a la referencia en cada iteración

float pitch_referencia = 0.0;                     // Orientación deseada (pitch)
float pitch_horizontal = 0.0;                     // Orientación deseada horizontal
float pitch_inclinado = 40.0;                     // Orientación deseada inclinada, para coger la comida
int umbral_pitch_1 = 45;                          // Inclinación a la que se cambia de pitch_horizontal a pitch_inclinado 
int umbral_pitch_2 = 37;                          // Inclinación a la que se cambia de pitch_inclinado a pitch_horizontal
float n_iter_pitch = 80.0;                        // Número de iteraciones en las que se recalcula la referencia

void setup() {
  Wire.begin();
  
  Serial.begin(9600);
  // Esperar a que el puerto serial esté listo
  delay(5000);

  // Iniciar la IMU
  if (!bno.begin()) {
    Serial.println("Failed to initialize IMU");
    while (true);
  }
 
  pinMode(LED_BUILTIN, OUTPUT);
  Servo1.attach(10);  
  Servo2.attach(11);  
  Servo3.attach(12);

  // Rutina de calibracion
  uint8_t system, gyro, accel, mg = 0; 
  while(system!=3){
    // El programa permanece en este bucle hasta que el sensor esté correctamente calibrado.
    bno.getCalibration(&system, &gyro, &accel, &mg);
    Serial.println("Calibrating");
  }
  Serial.println("Fully calibrated!!");
}

void loop() {
  
  float tiempo1 = millis();
  
  // Obtener la medida del sensor y calcular la temperatura
  float temperatura_read = analogRead(LM35);
  float temperatura = (3300 * temperatura_read*0.1)/1023.0;
  // Si la temperatura supera 36 grados el LED se enciende
  if(temperatura>36){
    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    digitalWrite(LED_BUILTIN, LOW);  
  }

  // Obtener orientación absoluta del sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  roll = euler.y();
  pitch = euler.z();
  heading = euler.x();

  // Recalcular la referencia de heading
  if(contador < n_iter_heading + 1){
    heading_referencia += paso;
    contador++;
    // Asignar valores negativos a los ángulos en los casos necesarios
    if(heading_referencia > 270){
      heading_referencia = heading_referencia - 360;
    }else if(heading_referencia < -90){
      heading_referencia = heading_referencia + 360;
    }
  }
  // Asignar valores negativos a los ángulos en los casos necesarios
  if((heading_referencia > 270)||(heading_referencia < 90)){
    if(heading > 180){
      heading = heading-360;
    }
  }
  // Calcular la desviación de heading respecto de la referencia
  diferencia = heading - heading_referencia;
  // Comprobar si dicha desviación supera al umbral y fijar la cantidad a añadir en la siguiente iteración
  if(diferencia > umbral_heading){
    contador = 1;
    paso = umbral_heading/n_iter_heading;
  }else if(diferencia < (-umbral_heading)){
    contador = 1;
    paso = -umbral_heading/n_iter_heading; 
  }

  // Comprobar la inclinación de la cuchara y recalcular la referencia si es necesario
  if(pitch > umbral_pitch_1){
    if(pitch_referencia <= pitch_inclinado){
      pitch_referencia += (pitch_inclinado/n_iter_pitch);
    }
  }else if(pitch < umbral_pitch_2){
    if(pitch_referencia >= pitch_horizontal){
      pitch_referencia-=(pitch_inclinado/n_iter_pitch);
    }
  }

  // Definir nueva orientación de los servos
  Servo1.write(93 + (pitch - pitch_referencia));
  Servo2.write(roll + 95);
  Servo3.write(90 + (heading - heading_referencia));

  // Esperar a que una nueva muestra esté disponible
  float tiempo_ejecucion = millis()- tiempo1;
  delay(BNO055_SAMPLERATE_DELAY_MS-tiempo_ejecucion);
}
