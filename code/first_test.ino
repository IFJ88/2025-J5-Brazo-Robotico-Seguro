// ===== Control de 5 servos con selección por botones y potenciómetro (ESP32) =====
// Librería: ESP32Servo (Arduino Library Manager)

#include <ESP32Servo.h>

// ----------- Pines -----------
const int BTN_NEXT = 13;   // botón a GND (usa INPUT_PULLUP)
const int BTN_PREV = 14;   // botón a GND (usa INPUT_PULLUP)
const int POT_PIN  = 34;   // potenciómetro (ADC1)

const int SERVO_COUNT = 5;
const int servoPins[SERVO_COUNT]  = {26, 19, 25, 18, 23};     // según tu mapeo
const int servoMinUs[SERVO_COUNT] = {400, 400, 500, 400, 400};
const int servoMaxUs[SERVO_COUNT] = {2600,2600,2600,2200,2600};

Servo servos[SERVO_COUNT];

// ----------- Estado -----------
int activeServo = 0;
int lastAngleWritten[SERVO_COUNT] = {90,90,90,90,90};

// ----------- Debounce -----------
unsigned long lastDebounceNext = 0, lastDebouncePrev = 0;
const unsigned long DEBOUNCE_MS = 35;
int lastRawNext = HIGH, lastStableNext = HIGH;
int lastRawPrev = HIGH, lastStablePrev = HIGH;

// ----------- Filtro pot -----------
float filt = 0.0f;
const float ALPHA = 0.15f;  // 0..1 (más alto = menos suavizado)

int adcToAngle(int adc) {
  if (adc < 0) adc = 0;
  if (adc > 4095) adc = 4095;
  int ang = map(adc, 0, 4095, 0, 180);
  if (ang < 0) ang = 0;
  if (ang > 180) ang = 180;
  return ang;
}

bool fallingEdgeDebounced(int pin, int &lastRaw, int &lastStable, unsigned long &tLast) {
  int raw = digitalRead(pin);
  unsigned long now = millis();

  if (raw != lastRaw) { lastRaw = raw; tLast = now; }
  if ((now - tLast) > DEBOUNCE_MS) {
    if (raw != lastStable) {
      lastStable = raw;
      if (lastStable == LOW) return true; // flanco de bajada (pulso a GND)
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);

  // Adjunta cada servo con su min/max propio
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(servoPins[i], servoMinUs[i], servoMaxUs[i]);
    servos[i].write(lastAngleWritten[i]); // posición inicial 90°
  }

  int raw = analogRead(POT_PIN);
  filt = (float)raw;

  Serial.println("Listo: selección con GPIO12/GPIO14, pot en GPIO34.");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.printf("Servo %d -> GPIO%d, min=%dus max=%dus\n",
                  i+1, servoPins[i], servoMinUs[i], servoMaxUs[i]);
  }
}

void loop() {
  // Navegación de servo activo
  if (fallingEdgeDebounced(BTN_NEXT, lastRawNext, lastStableNext, lastDebounceNext)) {
    activeServo = (activeServo + 1) % SERVO_COUNT;
    Serial.printf("Servo activo -> %d (GPIO%d)\n", activeServo+1, servoPins[activeServo]);
  }
  if (fallingEdgeDebounced(BTN_PREV, lastRawPrev, lastStablePrev, lastDebouncePrev)) {
    activeServo = (activeServo - 1 + SERVO_COUNT) % SERVO_COUNT;
    Serial.printf("Servo activo -> %d (GPIO%d)\n", activeServo+1, servoPins[activeServo]);
  }

  // Lectura y filtrado del potenciómetro
  int raw = analogRead(POT_PIN);
  filt = ALPHA * raw + (1.0f - ALPHA) * filt;
  int angle = adcToAngle((int)filt);

  // Escribe solo si cambia al menos 1°
  if (abs(angle - lastAngleWritten[activeServo]) >= 1) {
    servos[activeServo].write(angle); // 0..180 se mapea a min/max de ese servo
    lastAngleWritten[activeServo] = angle;
  }

  // Telemetría liviana
  static unsigned long t0 = 0;
  if (millis() - t0 > 500) {
    t0 = millis();
    Serial.printf("Activo #%d (GPIO%d) -> %d° | ADC=%d\n",
                  activeServo+1, servoPins[activeServo], angle, raw);
  }
}