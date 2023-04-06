#include <QTRSensors.h>

// Define os pinos dos sensores
#define SENSOR_PINS {A0, A1, A2, A3, A4, A5, A6, A7}
#define NUM_SENSORS 8

// Define as configurações do sensor de infravermelho
#define TIMEOUT 2500
#define EMITTER_PIN 2
#define THRESHOLD 500

// Define os pinos dos motores e a velocidade máxima
#define MOTOR_LEFT_PIN 3
#define MOTOR_RIGHT_PIN 4
#define MOTOR_SPEED_MAX 255

// Instancia o objeto QTRSensors com as configurações definidas
QTRSensors sensors(SENSOR_PINS, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

void setup() {
  // Define os pinos dos motores como saídas
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);

  // Desliga os motores
  digitalWrite(MOTOR_LEFT_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_PIN, LOW);

  // Aguarda meio segundo antes de iniciar a execução do loop
  delay(500);
}

void loop() {
  // Lê os valores dos sensores de infravermelho
  unsigned int sensorValues[NUM_SENSORS];
  sensors.read(sensorValues);

  // Calcula a posição do oponente com base nos valores dos sensores
  int position = sensors.readLine(sensorValues);

  // Calcula o erro de posição em relação ao centro da arena
  int error = position - (NUM_SENSORS - 1) / 2;

  // Calcula as velocidades dos motores esquerdo e direito com base no erro de posição
  int motorLeftSpeed = MOTOR_SPEED_MAX - error;
  int motorRightSpeed = MOTOR_SPEED_MAX + error;

  // Verifica se algum dos sensores detectou o oponente muito próximo
  if (sensorValues[0] > THRESHOLD || sensorValues[NUM_SENSORS - 1] > THRESHOLD) {
    // Ativa ambos os motores em alta velocidade por meio segundo para empurrar o oponente para fora da arena
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
    delay(500);

    // Desliga ambos os motores
    digitalWrite(MOTOR_LEFT_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
  } else {
    // Aplica as velocidades calculadas aos motores esquerdo e direito
    analogWrite(MOTOR_LEFT_PIN, motorLeftSpeed);
    analogWrite(MOTOR_RIGHT_PIN, motorRightSpeed);
  }
}
