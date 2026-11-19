#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// PINOS
const int pinSW420 = 2;
const int releSirene = 3;
const int releLampada = 4;

// TEMPOS
const unsigned long tempoSirene = 3000;   // ajuste manual
const unsigned long tempoLampada = 6000; // 6 segundos

// LIMIARES DE INCLINAÇÃO (em graus) – agora valores reais
const float anguloMaximoX = 7.0;   // limite para inclinação no eixo X (roll)
const float anguloMaximoY = 7.0;   // limite para inclinação no eixo Y (pitch)

MPU6050 mpu;
float referenciaX = 0, referenciaY = 0;  // ângulos médios de referência (graus)
bool mpuOk = false;

void setup() {
  pinMode(pinSW420, INPUT);      // vibração
  pinMode(releSirene, OUTPUT);
  pinMode(releLampada, OUTPUT);
  digitalWrite(releSirene, HIGH);
  digitalWrite(releLampada, HIGH);

  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    mpuOk = true;

    // Configura escala do acelerômetro para ±2g (maior precisão)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    // Calibração: média de 10 amostras dos ângulos (usa X, Y e Z)
    float sx = 0, sy = 0;
    for (int i = 0; i < 10; i++) {
      int16_t ax, ay, az;
      mpu.getAcceleration(&ax, &ay, &az);
      float ax_g = ax / 16384.0;
      float ay_g = ay / 16384.0;
      float az_g = az / 16384.0;

      float roll  = atan2(ay_g, az_g) * 180.0 / PI;
      float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

      sx += roll;
      sy += pitch;
      delay(5);
    }
    referenciaX = sx / 10.0;
    referenciaY = sy / 10.0;
  }
}

void loop() {
  static unsigned long tempoInicio = 0;
  static int estado = 0; // 0=espera, 1=alarme, 2=bloqueio

  switch (estado) {
    case 0: { // ESPERA
      // Leitura de vibração
      bool vib = digitalRead(pinSW420);

      // Leitura de inclinação (usa X, Y e Z no cálculo)
      bool inc = false;
      if (mpuOk) {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);
        float ax_g = ax / 16384.0;
        float ay_g = ay / 16384.0;
        float az_g = az / 16384.0;

        float roll  = atan2(ay_g, az_g) * 180.0 / PI;
        float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

        // Verifica diferenças em relação à referência
        if (fabs(roll - referenciaX) > anguloMaximoX ||
            fabs(pitch - referenciaY) > anguloMaximoY) {
          inc = true;
        }
      }

      if (vib || inc) {
        estado = 1;
        tempoInicio = millis();
        digitalWrite(releSirene, LOW);
        digitalWrite(releLampada, LOW);
      }
      break;
    }

    case 1: { // ALARME ATIVO
      unsigned long decorrido = millis() - tempoInicio;

      // Sirene: tempo ajustável manualmente
      if (decorrido < tempoSirene) {
        digitalWrite(releSirene, LOW);
      } else {
        digitalWrite(releSirene, HIGH);
      }

      // Lâmpada: pisca 6s (500ms liga/desliga)
      if ((decorrido % 1000) < 500) {
        digitalWrite(releLampada, LOW);
      } else {
        digitalWrite(releLampada, HIGH);
      }

      if (decorrido >= tempoLampada) {
        digitalWrite(releSirene, HIGH);
        digitalWrite(releLampada, HIGH);
        estado = 2;
        tempoInicio = millis();
      }
      break;
    }

    case 2: { // BLOQUEIO (5 segundos)
      if (millis() - tempoInicio >= 10000) {
        estado = 0;
      }
      break;
    }
  }

  delay(1);
}