/*
 * Firmware Arduino para Raspberry Pi Pico (RP2040)
 * Robot movil: 4 motores DC CQR37D 210:1 + BTS7960 + encoders cuadratura + FlySky iBUS
 *
 * Board package: "Raspberry Pi Pico/RP2040" by Earle Philhower
 *   https://github.com/earlephilhower/arduino-pico
 * Libreria requerida: ArduinoJson >= 7.x  (instalar desde Library Manager)
 *
 * Pinout:
 *   Motores BTS7960 (RPWM=adelante, LPWM=atras, EN=habilitar driver):
 *     M1(FL): RPWM=GP0,  LPWM=GP1,  EN=GP8
 *     M2(FR): RPWM=GP2,  LPWM=GP3,  EN=GP9
 *     M3(RL): RPWM=GP4,  LPWM=GP5,  EN=GP10
 *     M4(RR): RPWM=GP6,  LPWM=GP7,  EN=GP11
 *
 *   Encoders cuadratura X4 (ambos flancos de A y B, pull-up interno):
 *     E1(FL): A=GP16, B=GP17
 *     E2(FR): A=GP18, B=GP19
 *     E3(RL): A=GP20, B=GP21
 *     E4(RR): A=GP26, B=GP27
 *
 *   iBUS receptor FlySky IA6B:
 *     Serial1 (UART0) RX=GP13, TX=GP12 (no conectado)
 *
 *   Jetson Nano: Serial (USB CDC, 115200)
 *
 * Protocolo Jetson -> Pico (una linea JSON):
 *   {"cmd":"drive","vx":0.5,"omega":0.3}
 *   {"cmd":"stop"}
 *
 * Protocolo Pico -> Jetson (una linea JSON, cada 50 ms):
 *   {"enc":[FL,FR,RL,RR],"mode":"RC","ch":[ch1,ch2,ch3,ch4,ch5,ch6]}
 *
 * Motor CQR37D 210:1: 64 CPR en eje motor → 13440 pulsos/vuelta en eje salida (X4)
 */

#include <ArduinoJson.h>

// =============================================================================
// PINES
// =============================================================================

// Motores (BTS7960) — RPWM, LPWM, EN
constexpr int FL_RPWM = 0,  FL_LPWM = 1,  FL_EN = 8;
constexpr int FR_RPWM = 2,  FR_LPWM = 3,  FR_EN = 9;
constexpr int RL_RPWM = 4,  RL_LPWM = 5,  RL_EN = 10;
constexpr int RR_RPWM = 6,  RR_LPWM = 7,  RR_EN = 11;

// Encoders (cuadratura X4)
constexpr int ENC_FL_A = 16, ENC_FL_B = 17;
constexpr int ENC_FR_A = 18, ENC_FR_B = 19;
constexpr int ENC_RL_A = 20, ENC_RL_B = 21;
constexpr int ENC_RR_A = 26, ENC_RR_B = 27;

// =============================================================================
// CONSTANTES
// =============================================================================

constexpr int   PWM_FREQ          = 10000;  // Hz
constexpr int   PWM_RANGE         = 255;
constexpr int   CH_CENTER         = 1500;
constexpr int   CH_DEADZONE       = 50;
constexpr int   CH_MODE_IDX       = 5;      // canal 6 (0-indexed): RC/GCS
constexpr int   IBUS_PACKET_LEN   = 32;
constexpr int   IBUS_NUM_CHANNELS = 14;
constexpr int   IBUS_HEADER_0     = 0x20;
constexpr int   IBUS_HEADER_1     = 0x40;
constexpr unsigned long REPORT_INTERVAL_MS = 50;
constexpr unsigned long GCS_WATCHDOG_MS    = 500;
constexpr unsigned long IBUS_TIMEOUT_MS    = 500;

// Motor CQR37D 210:1 — 64 CPR en eje motor x 210 x 4 flancos = 13440 pulsos/vuelta
constexpr long ENCODER_CPR = 13440;

// =============================================================================
// ENCODERS (volatile, modificados en ISR)
//
// Decodificacion X4: ISR en CHANGE de canal A y canal B
//   ISR_A: si (A != B) → adelante (+1), si (A == B) → atras (-1)
//   ISR_B: si (A == B) → adelante (+1), si (A != B) → atras (-1)
// =============================================================================

volatile long encCounts[4] = {0, 0, 0, 0};

// FL
void isrFL_A() { encCounts[0] += (digitalRead(ENC_FL_A) != digitalRead(ENC_FL_B)) ? 1 : -1; }
void isrFL_B() { encCounts[0] += (digitalRead(ENC_FL_A) == digitalRead(ENC_FL_B)) ? 1 : -1; }
// FR
void isrFR_A() { encCounts[1] += (digitalRead(ENC_FR_A) != digitalRead(ENC_FR_B)) ? 1 : -1; }
void isrFR_B() { encCounts[1] += (digitalRead(ENC_FR_A) == digitalRead(ENC_FR_B)) ? 1 : -1; }
// RL
void isrRL_A() { encCounts[2] += (digitalRead(ENC_RL_A) != digitalRead(ENC_RL_B)) ? 1 : -1; }
void isrRL_B() { encCounts[2] += (digitalRead(ENC_RL_A) == digitalRead(ENC_RL_B)) ? 1 : -1; }
// RR
void isrRR_A() { encCounts[3] += (digitalRead(ENC_RR_A) != digitalRead(ENC_RR_B)) ? 1 : -1; }
void isrRR_B() { encCounts[3] += (digitalRead(ENC_RR_A) == digitalRead(ENC_RR_B)) ? 1 : -1; }

void setupEncoders() {
    int pins[] = {ENC_FL_A, ENC_FL_B, ENC_FR_A, ENC_FR_B,
                  ENC_RL_A, ENC_RL_B, ENC_RR_A, ENC_RR_B};
    for (int pin : pins) pinMode(pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_FL_B), isrFL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrFR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_B), isrFR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrRL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RL_B), isrRL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RR_B), isrRR_B, CHANGE);
}

// =============================================================================
// MOTORES
// =============================================================================

void setupMotors() {
    int pwmPins[] = {FL_RPWM, FL_LPWM, FR_RPWM, FR_LPWM,
                     RL_RPWM, RL_LPWM, RR_RPWM, RR_LPWM};
    int enPins[]  = {FL_EN, FR_EN, RL_EN, RR_EN};

    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_RANGE);

    for (int pin : pwmPins) { pinMode(pin, OUTPUT); analogWrite(pin, 0); }
    for (int pin : enPins)  { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }
}

// speed en [-1.0, 1.0]: positivo = adelante, negativo = atras
void motorSet(int rpwmPin, int lpwmPin, float speed) {
    speed = constrain(speed, -1.0f, 1.0f);
    int duty = (int)(fabs(speed) * PWM_RANGE);
    if (speed > 0.0f) {
        analogWrite(rpwmPin, duty); analogWrite(lpwmPin, 0);
    } else if (speed < 0.0f) {
        analogWrite(rpwmPin, 0);   analogWrite(lpwmPin, duty);
    } else {
        analogWrite(rpwmPin, 0);   analogWrite(lpwmPin, 0);
    }
}

void stopAll() {
    motorSet(FL_RPWM, FL_LPWM, 0);
    motorSet(FR_RPWM, FR_LPWM, 0);
    motorSet(RL_RPWM, RL_LPWM, 0);
    motorSet(RR_RPWM, RR_LPWM, 0);
}

// Conduccion diferencial tipo tanque
// vx en [-1,1] = velocidad lineal, omega en [-1,1] = giro (+ = horario)
void drive(float vx, float omega) {
    float left  = vx - omega;
    float right = vx + omega;
    float maxVal = max(max(fabs(left), fabs(right)), 1.0f);
    left  /= maxVal;
    right /= maxVal;

    motorSet(FL_RPWM, FL_LPWM, left);
    motorSet(RL_RPWM, RL_LPWM, left);
    motorSet(FR_RPWM, FR_LPWM, right);
    motorSet(RR_RPWM, RR_LPWM, right);
}

// =============================================================================
// iBUS RECEIVER (FlySky IA6B) — Serial1 (UART0) RX=GP13
//
// Paquete de 32 bytes:
//   [0]    0x20  (longitud)
//   [1]    0x40  (tipo: channel data)
//   [2..29] 14 canales x 2 bytes little-endian (1000-2000 us)
//   [30,31] checksum = 0xFFFF - suma(bytes 0..29), little-endian
// =============================================================================

int     ibusChannels[IBUS_NUM_CHANNELS];
unsigned long ibusLastUpdate = 0;
uint8_t ibusBuffer[IBUS_PACKET_LEN * 2];
int     ibusBufferLen = 0;

void setupIBUS() {
    for (int i = 0; i < IBUS_NUM_CHANNELS; i++) ibusChannels[i] = CH_CENTER;
    Serial1.setRX(13);
    Serial1.setTX(12);
    Serial1.begin(115200);
}

void ibusUpdate() {
    while (Serial1.available() && ibusBufferLen < (int)sizeof(ibusBuffer))
        ibusBuffer[ibusBufferLen++] = Serial1.read();

    while (ibusBufferLen >= IBUS_PACKET_LEN) {
        int startIdx = -1;
        for (int i = 0; i <= ibusBufferLen - 2; i++) {
            if (ibusBuffer[i] == IBUS_HEADER_0 && ibusBuffer[i+1] == IBUS_HEADER_1) {
                startIdx = i; break;
            }
        }

        if (startIdx < 0) {
            ibusBuffer[0] = ibusBuffer[ibusBufferLen - 1];
            ibusBufferLen = 1;
            break;
        }
        if (startIdx > 0) {
            ibusBufferLen -= startIdx;
            memmove(ibusBuffer, ibusBuffer + startIdx, ibusBufferLen);
            continue;
        }
        if (ibusBufferLen < IBUS_PACKET_LEN) break;

        uint16_t checksum = 0xFFFF;
        for (int i = 0; i < 30; i++) checksum -= ibusBuffer[i];
        uint16_t recvCs = ibusBuffer[30] | ((uint16_t)ibusBuffer[31] << 8);

        if (checksum == recvCs) {
            for (int ch = 0; ch < IBUS_NUM_CHANNELS; ch++)
                ibusChannels[ch] = ibusBuffer[2 + ch*2] | ((int)ibusBuffer[3 + ch*2] << 8);
            ibusLastUpdate = millis();
        }

        ibusBufferLen -= IBUS_PACKET_LEN;
        memmove(ibusBuffer, ibusBuffer + IBUS_PACKET_LEN, ibusBufferLen);
    }
}

bool ibusAlive() { return (millis() - ibusLastUpdate) < IBUS_TIMEOUT_MS; }

// Normaliza un canal a [-1.0, 1.0] con zona muerta
float ibusNormalized(int idx) {
    int val = ibusChannels[idx];
    int centered = val - CH_CENTER;
    if (abs(centered) < CH_DEADZONE) return 0.0f;
    int sign = (centered > 0) ? 1 : -1;
    float magnitude = (float)(abs(centered) - CH_DEADZONE) / (500.0f - CH_DEADZONE);
    return sign * constrain(magnitude, 0.0f, 1.0f);
}

// =============================================================================
// COMUNICACION CON JETSON (USB Serial)
// =============================================================================

String jetsonCmdBuf = "";

bool readJetsonLine(String &out) {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') { out = jetsonCmdBuf; jetsonCmdBuf = ""; return true; }
        jetsonCmdBuf += c;
    }
    return false;
}

void sendTelemetry(const char *mode) {
    noInterrupts();
    long enc[4] = {encCounts[0], encCounts[1], encCounts[2], encCounts[3]};
    interrupts();

    Serial.print(F("{\"enc\":["));
    Serial.print(enc[0]); Serial.print(',');
    Serial.print(enc[1]); Serial.print(',');
    Serial.print(enc[2]); Serial.print(',');
    Serial.print(enc[3]);
    Serial.print(F("],\"mode\":\""));
    Serial.print(mode);
    Serial.print(F("\",\"ch\":["));
    for (int i = 0; i < 6; i++) {
        Serial.print(ibusChannels[i]);
        if (i < 5) Serial.print(',');
    }
    Serial.println(F("]}"));
}

// =============================================================================
// ESTADO GLOBAL
// =============================================================================

float gcsVx    = 0.0f;
float gcsOmega = 0.0f;
unsigned long lastGcsCmd = 0;
unsigned long lastReport = 0;

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(10);

    setupMotors();
    setupEncoders();
    setupIBUS();

    lastGcsCmd = millis();
    lastReport = millis();

    Serial.println(F("Pico listo."));
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    unsigned long now = millis();

    ibusUpdate();

    bool rcAlive = ibusAlive();
    bool modeRC  = rcAlive && (ibusChannels[CH_MODE_IDX] < CH_CENTER);
    const char *modeStr = modeRC ? "RC" : "GCS";

    String line;
    if (readJetsonLine(line)) {
        line.trim();
        if (line.length() > 0) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, line);
            if (!err) {
                const char *cmd = doc["cmd"] | "";
                if (strcmp(cmd, "drive") == 0) {
                    gcsVx    = doc["vx"]    | 0.0f;
                    gcsOmega = doc["omega"] | 0.0f;
                    lastGcsCmd = now;
                } else if (strcmp(cmd, "stop") == 0) {
                    gcsVx = 0.0f; gcsOmega = 0.0f;
                }
            }
        }
    }

    if (modeRC) {
        float vx    = ibusNormalized(1);
        float omega = ibusNormalized(3);
        drive(vx, omega);
    } else {
        if (now - lastGcsCmd > GCS_WATCHDOG_MS) stopAll();
        else drive(gcsVx, gcsOmega);
    }

    if (now - lastReport >= REPORT_INTERVAL_MS) {
        sendTelemetry(modeStr);
        lastReport = now;
    }

    delay(5);
}
