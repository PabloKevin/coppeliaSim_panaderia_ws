/*
 * ============================================================================
 *  esp32_eezybot_linear.ino
 *  EEZYbotARM Mk2 real  --  ESP32 DevKit V1  --  Serial + MQTT
 * ============================================================================
 *  Los comandos llegan por Serial (115200) o por MQTT (topic MQTT_TOPIC_CMD);
 *  ambos usan EXACTAMENTE el mismo texto. Las respuestas salen por Serial y
 *  se publican en MQTT_TOPIC_STATUS.
 *
 *  ---------------------------------------------------------------------------
 *  EMPUJES PREDEFINIDOS (sin calibracion, en grados de servo)
 *    PUSH1   clase 1 (izquierda): HOME -> DEGS 130 170 120 0 -> HOME
 *    PUSH2   clase 2 (medio)    : HOME -> DEGS  80 160 130 0 -> HOME
 *    HOME    volver a la pose de reposo
 *  Todos los movimientos son suaves (perfil S-curve quintico, interpolando
 *  los grados de cada motor) y se encolan. Al terminar la secuencia se
 *  publica "IDLE". Los grados se editan en PUSH_POSE_DEG.
 *
 *  Desde la PC (con mosquitto):
 *    mosquitto_sub -h <broker> -t eezybot/status
 *    mosquitto_pub -h <broker> -t eezybot/cmd -m PUSH1
 *
 *  ---------------------------------------------------------------------------
 *  TODOS LOS COMANDOS
 *    HOME | PUSH1 | PUSH2 | PUSH n | STOP | POS | HELP
 *    DEGS b s e g     los 4 servos a esos grados (movimiento suave)
 *    DEG j g          servo j a g grados (instantaneo)   j: 0=base 1=hombro
 *    JOG j d          servo j +d grados (instantaneo)       2=codo 3=gripper
 *    VELJ v           velocidad maxima articular [deg/s]
 *    --- modo cartesiano (requiere calibracion, ver abajo) ---
 *    x y z | x y | MOVE x y z | OPEN | CLOSE | VEL v | ACC a | RANGE [z]
 *    SIM b s e g | RAW j us | CAL j q | DIR j | CALSHOW
 *
 *  ---------------------------------------------------------------------------
 *  HARDWARE
 *    GPIO13 base, GPIO14 hombro, GPIO27 codo, GPIO26 gripper.
 *    Servos con fuente externa 5-6 V (>= 3 A). El ESP32 puede alimentarse de
 *    esa misma fuente de 5 V por el pin VIN (NO por 3V3). GND comun.
 *    Recomendado: capacitor de 470-1000 uF en la linea de 5 V de los servos
 *    (los picos de corriente pueden reiniciar el ESP32).
 *
 *  Librerias: "ESP32Servo" (Kevin Harrington) y "PubSubClient" (Nick O'Leary).
 *  Placa: "DOIT ESP32 DEVKIT V1".
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <stdarg.h>

// ============================================================================
//  RED / MQTT   <<< COMPLETAR >>>
// ============================================================================
const char *WIFI_SSID     = "TU_WIFI";
const char *WIFI_PASS     = "TU_PASSWORD";
const char *MQTT_HOST     = "10.201.120.138";  // IP de la PC con el broker (hostname -I)
const uint16_t MQTT_PORT  = 1883;
const char *MQTT_USER     = nullptr;           // o "usuario" si el broker pide
const char *MQTT_PASSWD   = nullptr;
const char *MQTT_CLIENTID = "eezybot-esp32";
const char *MQTT_TOPIC_CMD    = "eezybot/cmd";
const char *MQTT_TOPIC_STATUS = "eezybot/status";
const char *MQTT_TOPIC_ONLINE = "eezybot/online";   // retenido: "1" / "0"

// ============================================================================
//  EMPUJES EN GRADOS DE SERVO   {base, hombro, codo, gripper}
// ============================================================================
const int N_PUSH = 2;
const float PUSH_POSE_DEG[N_PUSH][4] = {
  {130.0f, 170.0f, 120.0f, 0.0f},   // PUSH1 clase 1 (izquierda)
  { 80.0f, 160.0f, 130.0f, 0.0f},   // PUSH2 clase 2 (medio)
};
const float PUSH_HOLD_S = 0.20f;    // pausa en la pose de empuje antes de volver
const bool  PUSH_RETURN_HOME = true;

float jointVelDeg = 120.0f;         // deg/s   (VELJ)
const float JOINT_ACC_DEG = 600.0f; // deg/s^2

// ============================================================================
//  GEOMETRIA / CINEMATICA  (copiado de scripts/config.py)
// ============================================================================
const float L1 = 0.135f;
const float L2 = 0.147f;
const float L3 = 0.09447f;
const float L3_FIXED_ANGLE = -0.0990f;
const float H0 = 0.094f;
const float D0 = 0.000f;
const bool  ELBOW_UP = false;

const float WORLD_BASE_X   = -0.0438f;
const float WORLD_BASE_Y   =  0.0000f;
const float WORLD_BASE_Z   =  0.0000f;
const float WORLD_BASE_YAW =  0.0021f;

enum { J_BASE = 0, J_SHOULDER, J_ELBOW, J_GRIPPER, N_J };
const char *JNAME[N_J] = {"base", "shoulder", "elbow", "gripper"};

const float CAL_SIGN[N_J]   = {-1.0f, 1.0f, -1.0f, 1.0f};
const float CAL_OFFSET[N_J] = {0.0f, -1.8819885f, -0.0729548f, 0.0f};
const float LIM_LO[N_J] = {-3.1415f, -1.550f, -0.275f, 0.000f};
const float LIM_HI[N_J] = { 3.1415f,  0.356f,  0.705f, 1.450f};

const float GRIPPER_OPEN   = 0.8f;
const float GRIPPER_CLOSED = 0.0f;

// ============================================================================
//  MAPEO A SERVOS
// ============================================================================
const int SERVO_PIN[N_J] = {13, 14, 27, 26};
float SERVO_US_AT_ZERO[N_J] = {1500.0f, 1500.0f, 1500.0f, 1500.0f};
float SERVO_DIR[N_J]        = {1.0f, 1.0f, 1.0f, 1.0f};
float SERVO_US_PER_RAD[N_J] = {636.62f, 636.62f, 636.62f, 636.62f};
const int   SERVO_US_MIN = 500;
const int   SERVO_US_MAX = 2500;
const float SERVO_DEG_SPAN = 180.0f;    // 500us = 0 deg, 2500us = 180 deg

// ============================================================================
//  PERFIL CARTESIANO
// ============================================================================
float linVel = 0.10f;
float linAcc = 0.30f;
const float MAX_JOINT_VEL[N_J] = {3.0f, 3.0f, 3.0f, 6.0f};
const float MAX_JOINT_ACC[N_J] = {12.0f, 12.0f, 12.0f, 20.0f};

const float UPDATE_HZ         = 50.0f;
const float MIN_MOVE_DURATION = 0.05f;
const float GRIPPER_MOVE_TIME = 0.40f;
const int   N_CHECK           = 50;

// HOME (mundo, m). Con la calibracion por defecto equivale a ~ DEGS 90 89 115 136
const float HOME_X = 0.14f, HOME_Y = 0.0f, HOME_Z = 0.1425f;

const float PEAK_VEL_COEFF   = 1.875f;
const float PEAK_ACCEL_COEFF = 5.7735027f;

// ============================================================================
//  TIPOS Y ESTADO
// ============================================================================
struct Vec3 { float x, y, z; };

struct Move {
  bool  joint;              // true: interpolacion en pulsos; false: recta cartesiana
  Vec3  p0, p1;             // cartesiano
  float g0, g1;
  float u0[N_J], u1[N_J];   // articular (us)
  bool  endCartValid;       // articular: la pose final es cartesiana conocida
  Vec3  endPos;
  float endGrip;
  float T;
};

const int QUEUE_LEN = 16;
Move queueBuf[QUEUE_LEN];
int  qHead = 0, qCount = 0;

Move     active;
bool     moving = false;
uint32_t moveT0 = 0;

// Estado actual (lo que tienen los servos ahora)
Vec3  curPos;
float curGrip;
bool  curCartValid = false;
float lastCmd[N_J] = {0};
int   lastUs[N_J]  = {0};

// Estado al final de la cola
Vec3  tailPos;
float tailGrip;
float tailUs[N_J];
bool  poseValid = false;    // pose cartesiana conocida al final de la cola

float homeUs[N_J];

Servo servos[N_J];
uint32_t lastTickUs = 0;

char lineBuf[96];
int  lineLen = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
char  mqttCmd[96];
bool  mqttPending = false;
bool  wifiAnnounced = false;
uint32_t lastMqttTry = 0;

// ============================================================================
//  SALIDA: Serial + MQTT
// ============================================================================
void out(const char *fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
  Serial.print(buf);
  size_t n = strlen(buf);
  if (n == 0 || buf[n - 1] != '\n') Serial.println();
  while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = 0;
  if (n && mqtt.connected()) mqtt.publish(MQTT_TOPIC_STATUS, buf);
}

// ============================================================================
//  CONVERSIONES
// ============================================================================
static inline float quinticS(float tau) {
  return tau * tau * tau * (10.0f + tau * (-15.0f + 6.0f * tau));
}
float cmdToUs(int j, float q)  { return SERVO_US_AT_ZERO[j] + SERVO_DIR[j] * SERVO_US_PER_RAD[j] * q; }
float usToCmd(int j, float us) { return (us - SERVO_US_AT_ZERO[j]) / (SERVO_DIR[j] * SERVO_US_PER_RAD[j]); }
float usToDeg(float us) { return (us - SERVO_US_MIN) * SERVO_DEG_SPAN / (SERVO_US_MAX - SERVO_US_MIN); }
float degToUs(float d)  { return SERVO_US_MIN + d * (SERVO_US_MAX - SERVO_US_MIN) / SERVO_DEG_SPAN; }

Vec3 worldToLocal(const Vec3 &w) {
  float dx = w.x - WORLD_BASE_X, dy = w.y - WORLD_BASE_Y, dz = w.z - WORLD_BASE_Z;
  float c = cosf(-WORLD_BASE_YAW), s = sinf(-WORLD_BASE_YAW);
  return { dx * c - dy * s, dx * s + dy * c, dz };
}

static inline Vec3 lerp3(const Vec3 &a, const Vec3 &b, float s) {
  return { a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s };
}

// ============================================================================
//  CINEMATICA INVERSA
// ============================================================================
bool solveIK(const Vec3 &w, float grip, float cmd[N_J], char *err, size_t n) {
  Vec3 l = worldToLocal(w);
  float thBase = atan2f(l.y, l.x);
  float r  = hypotf(l.x, l.y) - D0;
  float zr = l.z - H0;
  float r2 = r  - L3 * cosf(L3_FIXED_ANGLE);
  float z2 = zr - L3 * sinf(L3_FIXED_ANGLE);
  float d  = hypotf(r2, z2);

  if (d < fabsf(L1 - L2) || d > (L1 + L2)) {
    snprintf(err, n, "fuera de alcance (%.3f,%.3f,%.3f) d=%.4f", w.x, w.y, w.z, d);
    return false;
  }
  float c = constrain((d * d - L1 * L1 - L2 * L2) / (2.0f * L1 * L2), -1.0f, 1.0f);
  float t2rel = acosf(c);
  if (!ELBOW_UP) t2rel = -t2rel;
  float th1 = atan2f(z2, r2) - atan2f(L2 * sinf(t2rel), L1 + L2 * cosf(t2rel));
  float th2abs = th1 + t2rel;

  float internal[N_J] = {thBase, th1, th2abs, grip};
  for (int j = 0; j < N_J; j++) {
    float q = CAL_SIGN[j] * internal[j] + CAL_OFFSET[j];
    if (q < LIM_LO[j] || q > LIM_HI[j]) {
      snprintf(err, n, "%s=%.3f rad fuera de limite [%.3f,%.3f] en (%.3f,%.3f,%.3f)",
               JNAME[j], q, LIM_LO[j], LIM_HI[j], w.x, w.y, w.z);
      return false;
    }
    float us = cmdToUs(j, q);
    if (us < SERVO_US_MIN || us > SERVO_US_MAX) {
      snprintf(err, n, "servo %s saturado (%.0f us) en (%.3f,%.3f,%.3f)", JNAME[j], us, w.x, w.y, w.z);
      return false;
    }
    cmd[j] = q;
  }
  return true;
}

void writeUs(const float us[N_J]) {
  for (int j = 0; j < N_J; j++) {
    int u = constrain((int)lroundf(us[j]), SERVO_US_MIN, SERVO_US_MAX);
    servos[j].writeMicroseconds(u);
    lastUs[j]  = u;
    lastCmd[j] = usToCmd(j, u);
  }
}

void writeServos(const float cmd[N_J]) {
  float us[N_J];
  for (int j = 0; j < N_J; j++) us[j] = cmdToUs(j, cmd[j]);
  writeUs(us);
  for (int j = 0; j < N_J; j++) lastCmd[j] = cmd[j];
}

// ============================================================================
//  PLANIFICADORES
// ============================================================================
bool planCartesian(const Vec3 &p0, float g0, const Vec3 &p1, float g1, Move &m, char *err, size_t n) {
  float dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);

  float T = 0.0f;
  if (dist > 1e-6f) T = max(PEAK_VEL_COEFF * dist / linVel, sqrtf(PEAK_ACCEL_COEFF * dist / linAcc));
  if (fabsf(g1 - g0) > 1e-6f) T = max(T, GRIPPER_MOVE_TIME);
  T = max(T, MIN_MOVE_DURATION);

  float dt = T / N_CHECK;
  float prev[N_J], prevV[N_J], q[N_J];
  float maxV[N_J] = {0}, maxA[N_J] = {0};
  bool havePrev = false, havePrevV = false;
  for (int i = 0; i <= N_CHECK; i++) {
    float s = quinticS((float)i / N_CHECK);
    if (!solveIK(lerp3(p0, p1, s), g0 + (g1 - g0) * s, q, err, n)) return false;
    if (havePrev) {
      float v[N_J];
      for (int j = 0; j < N_J; j++) {
        v[j] = (q[j] - prev[j]) / dt;
        maxV[j] = max(maxV[j], fabsf(v[j]));
        if (havePrevV) maxA[j] = max(maxA[j], fabsf((v[j] - prevV[j]) / dt));
      }
      memcpy(prevV, v, sizeof v);
      havePrevV = true;
    }
    memcpy(prev, q, sizeof q);
    havePrev = true;
  }
  float scale = 1.0f;
  for (int j = 0; j < N_J; j++) {
    if (maxV[j] > MAX_JOINT_VEL[j]) scale = max(scale, maxV[j] / MAX_JOINT_VEL[j]);
    if (maxA[j] > MAX_JOINT_ACC[j]) scale = max(scale, sqrtf(maxA[j] / MAX_JOINT_ACC[j]));
  }
  m.joint = false;
  m.p0 = p0; m.p1 = p1; m.g0 = g0; m.g1 = g1; m.T = T * scale;
  return true;
}

bool pushQueue(const Move &m) {
  if (qCount >= QUEUE_LEN) { out("ERR cola llena"); return false; }
  queueBuf[(qHead + qCount) % QUEUE_LEN] = m;
  qCount++;
  return true;
}

bool enqueueCart(const Vec3 &target, float grip) {
  if (!poseValid) { out("ERR pose cartesiana desconocida: mandar HOME primero"); return false; }
  Move m; char err[112];
  if (!planCartesian(tailPos, tailGrip, target, grip, m, err, sizeof err)) { out("ERR %s", err); return false; }
  float q[N_J];
  solveIK(target, grip, q, err, sizeof err);
  if (!pushQueue(m)) return false;
  for (int j = 0; j < N_J; j++) tailUs[j] = cmdToUs(j, q[j]);
  tailPos = target; tailGrip = grip;
  out("OK -> (%.4f, %.4f, %.4f) grip=%.2f  T=%.2fs  [cola %d]",
      target.x, target.y, target.z, grip, m.T, qCount);
  return true;
}

// Movimiento suave en espacio articular hacia los pulsos us1.
// endCart/endPos/endGrip: si la pose final es una pose cartesiana conocida (HOME).
bool enqueueJoint(const float us1[N_J], float minT, bool endCart, const Vec3 &endPos,
                  float endGrip, const char *label) {
  Move m;
  m.joint = true;
  float T = 0.0f;
  for (int j = 0; j < N_J; j++) {
    m.u0[j] = tailUs[j];
    m.u1[j] = constrain(us1[j], (float)SERVO_US_MIN, (float)SERVO_US_MAX);
    float dDeg = fabsf(usToDeg(m.u1[j]) - usToDeg(m.u0[j]));
    if (dDeg > 1e-3f)
      T = max(T, max(PEAK_VEL_COEFF * dDeg / jointVelDeg, sqrtf(PEAK_ACCEL_COEFF * dDeg / JOINT_ACC_DEG)));
  }
  m.T = max(max(T, minT), MIN_MOVE_DURATION);
  m.endCartValid = endCart; m.endPos = endPos; m.endGrip = endGrip;
  if (!pushQueue(m)) return false;

  for (int j = 0; j < N_J; j++) tailUs[j] = m.u1[j];
  poseValid = endCart;
  if (endCart) { tailPos = endPos; tailGrip = endGrip; }
  out("OK %s -> DEGS %.0f %.0f %.0f %.0f  T=%.2fs  [cola %d]", label,
      usToDeg(m.u1[0]), usToDeg(m.u1[1]), usToDeg(m.u1[2]), usToDeg(m.u1[3]), m.T, qCount);
  return true;
}

void stopAll() {
  moving = false;
  qCount = 0;
  for (int j = 0; j < N_J; j++) tailUs[j] = lastUs[j];
  poseValid = curCartValid;
  tailPos = curPos; tailGrip = curGrip;
}

void motionTick() {
  if (!moving) {
    if (qCount == 0) return;
    active = queueBuf[qHead];
    qHead = (qHead + 1) % QUEUE_LEN;
    qCount--;
    moving = true;
    moveT0 = micros();
  }

  float t = (micros() - moveT0) * 1e-6f;
  bool done = t >= active.T;
  float s = done ? 1.0f : quinticS(t / active.T);

  if (active.joint) {
    float us[N_J];
    for (int j = 0; j < N_J; j++) us[j] = active.u0[j] + (active.u1[j] - active.u0[j]) * s;
    writeUs(us);
    curCartValid = false;
    if (done && active.endCartValid) {
      curCartValid = true; curPos = active.endPos; curGrip = active.endGrip;
    }
  } else {
    Vec3  p = lerp3(active.p0, active.p1, s);
    float g = active.g0 + (active.g1 - active.g0) * s;
    float cmd[N_J]; char err[112];
    if (!solveIK(p, g, cmd, err, sizeof err)) {
      out("ERR en movimiento: %s -> STOP", err);
      stopAll();
      return;
    }
    writeServos(cmd);
    curPos = p; curGrip = g; curCartValid = true;
  }

  if (done) {
    moving = false;
    out("DONE");
    if (qCount == 0) out("IDLE");
  }
}

// ============================================================================
//  ACCIONES
// ============================================================================
const Vec3 HOME_POS = {HOME_X, HOME_Y, HOME_Z};

void goHome() {
  if (poseValid) enqueueCart(HOME_POS, GRIPPER_OPEN);
  else           enqueueJoint(homeUs, 0.0f, true, HOME_POS, GRIPPER_OPEN, "HOME");
}

void doPush(int n) {
  if (n < 1 || n > N_PUSH) { out("ERR PUSH: clase %d no existe (1..%d)", n, N_PUSH); return; }
  int needed = 2 + (PUSH_HOLD_S > 0 ? 1 : 0) + (PUSH_RETURN_HOME ? 1 : 0);
  if (qCount + needed > QUEUE_LEN) { out("ERR cola llena"); return; }

  float pushUs[N_J];
  for (int j = 0; j < N_J; j++) pushUs[j] = degToUs(PUSH_POSE_DEG[n - 1][j]);

  out("PUSH%d inicio", n);
  enqueueJoint(homeUs, 0.0f, true, HOME_POS, GRIPPER_OPEN, "HOME");
  enqueueJoint(pushUs, 0.0f, false, HOME_POS, 0.0f, n == 1 ? "PUSH1" : "PUSH2");
  if (PUSH_HOLD_S > 0) enqueueJoint(pushUs, PUSH_HOLD_S, false, HOME_POS, 0.0f, "hold");
  if (PUSH_RETURN_HOME) enqueueJoint(homeUs, 0.0f, true, HOME_POS, GRIPPER_OPEN, "HOME");
}

// Escribe un pulso directo a un servo (instantaneo, sin cinematica)
void writeRawUs(int j, float us) {
  stopAll();
  float u[N_J];
  for (int k = 0; k < N_J; k++) u[k] = lastUs[k];
  u[j] = us;
  writeUs(u);
  for (int k = 0; k < N_J; k++) tailUs[k] = lastUs[k];
  curCartValid = poseValid = false;
}

// ============================================================================
//  COMANDOS
// ============================================================================
void printHelp() {
  static const char *lines[] = {
    "=== EEZYbotARM Mk2 (Serial + MQTT) ===",
    " HOME | PUSH1 | PUSH2 | STOP | POS | HELP",
    " DEGS b s e g  4 servos en grados (suave)   VELJ v  vel. articular [deg/s]",
    " DEG j g | JOG j d   servo j directo (0=base 1=hombro 2=codo 3=gripper)",
    " cartesiano: x y z | x y | OPEN | CLOSE | VEL v | ACC a | RANGE [z]",
    " calibracion: SIM b s e g | RAW j us | CAL j q | DIR j | CALSHOW",
  };
  for (auto l : lines) out("%s", l);
}

void printPos() {
  out("POS %sxyz=(%.4f, %.4f, %.4f) %s", curCartValid ? "" : "(cartesiana desconocida) ",
      curPos.x, curPos.y, curPos.z, moving ? "[moviendo]" : "");
  out("DEGS %.1f %.1f %.1f %.1f  (us %d %d %d %d)  cola=%d",
      usToDeg(lastUs[0]), usToDeg(lastUs[1]), usToDeg(lastUs[2]), usToDeg(lastUs[3]),
      lastUs[0], lastUs[1], lastUs[2], lastUs[3], qCount);
}

void printCal() {
  out("float SERVO_US_AT_ZERO[N_J] = {%.1ff, %.1ff, %.1ff, %.1ff};",
      SERVO_US_AT_ZERO[0], SERVO_US_AT_ZERO[1], SERVO_US_AT_ZERO[2], SERVO_US_AT_ZERO[3]);
  out("float SERVO_DIR[N_J]        = {%.1ff, %.1ff, %.1ff, %.1ff};",
      SERVO_DIR[0], SERVO_DIR[1], SERVO_DIR[2], SERVO_DIR[3]);
}

void printRange(float z) {
  float xmin = NAN, xmax = NAN, q[N_J]; char e[112];
  for (int i = 0; i <= 400; i++) {
    float x = i * 0.001f;
    if (solveIK({x, 0.0f, z}, GRIPPER_OPEN, q, e, sizeof e)) { if (isnan(xmin)) xmin = x; xmax = x; }
  }
  if (isnan(xmin)) out("RANGE z=%.4f: nada alcanzable con y=0", z);
  else out("RANGE z=%.4f: x de %.3f a %.3f m (y=0)", z, xmin, xmax);
}

bool isNumberToken(const char *t) {
  return t && (isdigit((unsigned char)t[0]) || t[0] == '-' || t[0] == '+' || t[0] == '.');
}

void handleLine(char *line) {
  for (char *c = line; *c; c++) if (*c == ',' || *c == ';' || *c == '\t') *c = ' ';

  char *tok[6]; int nt = 0;
  for (char *t = strtok(line, " "); t && nt < 6; t = strtok(NULL, " ")) tok[nt++] = t;
  if (nt == 0) return;

  int argStart = 0;
  char cmd[12] = "MOVE";
  if (!isNumberToken(tok[0])) {
    strncpy(cmd, tok[0], sizeof cmd - 1); cmd[sizeof cmd - 1] = 0;
    for (char *c = cmd; *c; c++) *c = toupper((unsigned char)*c);
    argStart = 1;
  }
  int na = nt - argStart;
  float a[5] = {0};
  for (int i = 0; i < na && i < 5; i++) {
    char *end;
    a[i] = strtof(tok[argStart + i], &end);
    if (end == tok[argStart + i]) { out("ERR argumento invalido '%s'", tok[argStart + i]); return; }
  }
  bool jOk = na >= 1 && a[0] >= 0 && a[0] < N_J;

  if (!strcmp(cmd, "HOME"))  goHome();
  else if (!strncmp(cmd, "PUSH", 4)) doPush(cmd[4] ? atoi(cmd + 4) : (na ? (int)a[0] : 0));
  else if (!strcmp(cmd, "STOP"))  { stopAll(); out("OK STOP"); printPos(); }
  else if (!strcmp(cmd, "POS"))   printPos();
  else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "DEGS") && na == 4) {
    float us[N_J];
    for (int j = 0; j < N_J; j++) us[j] = degToUs(constrain(a[j], 0.0f, SERVO_DEG_SPAN));
    enqueueJoint(us, 0.0f, false, HOME_POS, 0.0f, "DEGS");
  }
  else if (!strcmp(cmd, "DEG") && na == 2 && jOk) {
    int j = (int)a[0];
    writeRawUs(j, degToUs(constrain(a[1], 0.0f, SERVO_DEG_SPAN)));
    out("OK DEG %s = %.1f deg (%d us)", JNAME[j], usToDeg(lastUs[j]), lastUs[j]);
  }
  else if (!strcmp(cmd, "JOG") && na == 2 && jOk) {
    int j = (int)a[0];
    writeRawUs(j, degToUs(constrain(usToDeg(lastUs[j]) + a[1], 0.0f, SERVO_DEG_SPAN)));
    out("OK JOG %s -> %.1f deg (%d us)", JNAME[j], usToDeg(lastUs[j]), lastUs[j]);
  }
  else if (!strcmp(cmd, "VELJ") && na == 1 && a[0] > 0.0f && a[0] <= 600.0f) {
    jointVelDeg = a[0]; out("OK VELJ %.0f deg/s", jointVelDeg);
  }
  // ---------------- modo cartesiano ----------------
  else if (!strcmp(cmd, "MOVE")) {
    if (na == 3)      enqueueCart({a[0], a[1], a[2]}, tailGrip);
    else if (na == 2) enqueueCart({a[0], a[1], tailPos.z}, tailGrip);
    else out("ERR uso: x y z  |  x y");
  }
  else if (!strcmp(cmd, "OPEN"))  enqueueCart(tailPos, GRIPPER_OPEN);
  else if (!strcmp(cmd, "CLOSE")) enqueueCart(tailPos, GRIPPER_CLOSED);
  else if (!strcmp(cmd, "VEL") && na == 1 && a[0] > 0.0f && a[0] <= 1.0f) {
    linVel = a[0]; out("OK VEL %.3f m/s", linVel);
  }
  else if (!strcmp(cmd, "ACC") && na == 1 && a[0] > 0.0f && a[0] <= 5.0f) {
    linAcc = a[0]; out("OK ACC %.3f m/s^2", linAcc);
  }
  else if (!strcmp(cmd, "RANGE") && na <= 1) printRange(na == 1 ? a[0] : tailPos.z);
  // ---------------- calibracion ----------------
  else if (!strcmp(cmd, "SIM") && na == 4) {
    float q[N_J];
    for (int j = 0; j < N_J; j++) q[j] = constrain(a[j], LIM_LO[j], LIM_HI[j]);
    stopAll(); writeServos(q);
    for (int j = 0; j < N_J; j++) tailUs[j] = lastUs[j];
    curCartValid = poseValid = false;
    out("OK SIM"); printPos();
  }
  else if (!strcmp(cmd, "RAW") && na == 2 && jOk) {
    int j = (int)a[0];
    writeRawUs(j, a[1]);
    out("OK RAW %s = %d us = %.1f deg (q_sim %+.4f rad)", JNAME[j], lastUs[j], usToDeg(lastUs[j]), lastCmd[j]);
  }
  else if (!strcmp(cmd, "CAL") && na == 2 && jOk) {
    int j = (int)a[0];
    SERVO_US_AT_ZERO[j] = lastUs[j] - SERVO_DIR[j] * SERVO_US_PER_RAD[j] * a[1];
    lastCmd[j] = a[1];
    out("OK CAL %s: %d us <-> %+.4f rad (solo en RAM)", JNAME[j], lastUs[j], a[1]);
    printCal();
  }
  else if (!strcmp(cmd, "DIR") && na == 1 && jOk) {
    int j = (int)a[0];
    float q = lastCmd[j];
    SERVO_DIR[j] = -SERVO_DIR[j];
    SERVO_US_AT_ZERO[j] = lastUs[j] - SERVO_DIR[j] * SERVO_US_PER_RAD[j] * q;
    out("OK DIR %s = %+.0f (solo en RAM)", JNAME[j], SERVO_DIR[j]);
    printCal();
  }
  else if (!strcmp(cmd, "CALSHOW")) printCal();
  else out("ERR comando desconocido / argumentos invalidos (HELP)");
}

// ============================================================================
//  ENTRADAS: Serial y MQTT
// ============================================================================
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// PubSubClient reutiliza su buffer para publicar: copiar y procesar fuera del callback
void onMqttMessage(char *topic, byte *payload, unsigned int len) {
  len = min(len, (unsigned int)sizeof(mqttCmd) - 1);
  memcpy(mqttCmd, payload, len);
  mqttCmd[len] = 0;
  mqttPending = true;
}

void netTick() {
  if (WiFi.status() != WL_CONNECTED) { wifiAnnounced = false; return; }
  if (!wifiAnnounced) {
    wifiAnnounced = true;
    Serial.printf("WiFi conectado, IP %s\n", WiFi.localIP().toString().c_str());
  }

  if (!mqtt.connected()) {
    // connect() bloquea un poco: no intentar en medio de un movimiento
    if (moving || millis() - lastMqttTry < 3000) return;
    lastMqttTry = millis();
    if (mqtt.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWD, MQTT_TOPIC_ONLINE, 0, true, "0")) {
      mqtt.subscribe(MQTT_TOPIC_CMD);
      mqtt.publish(MQTT_TOPIC_ONLINE, "1", true);
      out("MQTT conectado a %s:%u, escuchando '%s'", MQTT_HOST, MQTT_PORT, MQTT_TOPIC_CMD);
    } else {
      IPAddress ip = WiFi.localIP(), gw = WiFi.gatewayIP(), mk = WiFi.subnetMask();
      Serial.printf("\nMQTT no conecta a %s:%u (rc=%d). ESP32: red '%s' IP %s gw %s mask %s RSSI %d dBm\n",
                    MQTT_HOST, MQTT_PORT, mqtt.state(), WiFi.SSID().c_str(),
                    ip.toString().c_str(), gw.toString().c_str(), mk.toString().c_str(), WiFi.RSSI());
      IPAddress host;
      if (host.fromString(MQTT_HOST) &&
          (uint32_t(host) & uint32_t(mk)) != (uint32_t(ip) & uint32_t(mk)))
        Serial.println("  -> la PC y la ESP32 estan en SUBREDES DISTINTAS: no estan en la misma red WiFi");
    }
    return;
  }

  mqtt.loop();
  if (mqttPending) {
    mqttPending = false;
    Serial.printf("[mqtt] %s\n", mqttCmd);
    handleLine(mqttCmd);
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int j = 0; j < N_J; j++) {
    servos[j].setPeriodHertz(50);
    servos[j].attach(SERVO_PIN[j], SERVO_US_MIN, SERVO_US_MAX);
  }

  // HOME directo al arrancar (no hay realimentacion de posicion)
  float q[N_J]; char err[112];
  if (!solveIK(HOME_POS, GRIPPER_OPEN, q, err, sizeof err)) {
    Serial.printf("ERR HOME invalido: %s\n", err);
  }
  for (int j = 0; j < N_J; j++) homeUs[j] = cmdToUs(j, q[j]);
  writeServos(q);
  curPos = tailPos = HOME_POS;
  curGrip = tailGrip = GRIPPER_OPEN;
  curCartValid = poseValid = true;
  for (int j = 0; j < N_J; j++) tailUs[j] = lastUs[j];

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(2);

  printHelp();
  printPos();
  lastTickUs = micros();
}

void loop() {
  readSerial();
  netTick();
  uint32_t now = micros();
  if (now - lastTickUs >= (uint32_t)(1e6f / UPDATE_HZ)) {
    lastTickUs = now;
    motionTick();
  }
}
