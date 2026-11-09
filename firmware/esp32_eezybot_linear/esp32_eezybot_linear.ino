/*
 * ============================================================================
 *  esp32_eezybot_linear.ino
 *  EEZYbotARM Mk2 real  --  ESP32 DevKit V1
 * ============================================================================
 *  Port a C++ de la cadena de control del gemelo digital
 *  (scripts/config.py + kinematics.py + trajectory.py):
 *
 *    coordenada (x,y,z) mundo  ->  FrameTransform.world_to_local
 *                              ->  InverseKinematics.solve
 *                              ->  JointCalibration.to_command   (rad "sim")
 *                              ->  mapeo servo (us)              (NUEVO, TUNE)
 *
 *  Movimiento: recta cartesiana entre la posicion actual y el objetivo, con
 *  perfil S-curve quintico (igual que LinearCartesianPlanner). La IK se
 *  resuelve en cada paso (50 Hz), asi el efector sigue la recta y mantiene la
 *  altura si el z de origen y destino es el mismo (empuje a altura constante).
 *  Antes de aceptar un movimiento se valida TODA la recta (alcanzable, dentro
 *  de limites de articulacion y de servo) y se estira la duracion si alguna
 *  articulacion superaria su velocidad/aceleracion maxima.
 *
 *  ---------------------------------------------------------------------------
 *  COMANDOS SERIE (115200 baud, terminar con Enter / '\n')
 *  Coordenadas en METROS, en el mismo marco "mundo" que CoppeliaSim
 *  (el que usan las rutinas JSON, p.ej. push_Middle.json).
 *  Se aceptan espacios, comas o ';' como separadores.
 *
 *    x y z            Mover en linea recta a (x,y,z).        ej: 0.20 0 0.1425
 *    x y              Mover en linea recta a (x,y) manteniendo la altura actual
 *    MOVE x y z       Igual que "x y z"
 *    HOME             Ir a HOME (en linea recta si la pose es conocida)
 *    OPEN / CLOSE     Abrir / cerrar gripper
 *    VEL v            Velocidad lineal maxima [m/s]          ej: VEL 0.08
 *    ACC a            Aceleracion lineal maxima [m/s^2]
 *    STOP             Frenar en el lugar y vaciar la cola
 *    POS              Imprimir posicion, comandos (rad) y pulsos (us)
 *    SIM b s e g      Escribir directo valores de articulacion "sim" (rad),
 *                     los mismos que ves en motor1..4 de CoppeliaSim (calibracion)
 *    RAW j us         Escribir pulso crudo a un servo (j = 0..3)  (calibracion)
 *    HELP             Ayuda
 *
 *  Los movimientos se encolan (hasta QUEUE_LEN), asi se puede mandar
 *  "origen" y luego "destino del empuje" seguidos.
 *  Respuestas: "OK ...", "ERR ...", "DONE" (fin de cada tramo), "IDLE" (cola vacia).
 *
 *  ---------------------------------------------------------------------------
 *  HARDWARE
 *    GPIO13 -> servo base      (motor1)
 *    GPIO14 -> servo hombro    (motor2)
 *    GPIO27 -> servo codo      (motor3)
 *    GPIO26 -> servo gripper   (motor4)
 *    Servos alimentados con fuente EXTERNA 5-6 V (>= 3 A). GND de la fuente
 *    unido al GND del ESP32. NO alimentar los MG996R desde el pin VIN/3V3.
 *
 *  Libreria: "ESP32Servo" (Kevin Harrington) desde el Library Manager.
 *  Placa: "DOIT ESP32 DEVKIT V1" (o "ESP32 Dev Module").
 *
 *  ---------------------------------------------------------------------------
 *  CALIBRACION DE SERVOS (hacer una vez, antes de mandar coordenadas)
 *    El modelo de CoppeliaSim ya esta calibrado: dado (x,y,z) da el valor de
 *    cada joint (rad). Solo falta saber que pulso del servo real corresponde
 *    a cada valor de joint:
 *        us = SERVO_US_AT_ZERO[j] + SERVO_DIR[j] * SERVO_US_PER_RAD[j] * q_sim
 *    1) Con los servos SIN cuernos/brazos montados (o con cuidado), mandar
 *       "RAW j us" hasta encontrar el pulso en que la articulacion queda igual
 *       que en CoppeliaSim con motorX = 0 rad  ->  SERVO_US_AT_ZERO[j].
 *    2) Mandar "SIM 0 0.3 0 0" (por ej.). Si el hombro se mueve al reves que
 *       en la sim, cambiar SERVO_DIR del hombro a -1. Repetir por articulacion.
 *    3) Verificar con "SIM ..." en varias poses comparando con la sim.
 *    4) Probar "HOME", luego "0.20 0 0.1425" (empuje recto de push_Middle).
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================================
//  GEOMETRIA / CINEMATICA  (copiado de scripts/config.py)
// ============================================================================
const float L1 = 0.135f;
const float L2 = 0.147f;
const float L3 = 0.09447f;
const float L3_FIXED_ANGLE = -0.0990f;   // rad
const float H0 = 0.094f;
const float D0 = 0.000f;
const bool  ELBOW_UP = false;            // ELBOW_CONFIG = "down"

// Marco mundo -> marco base del brazo
const float WORLD_BASE_X   = -0.0438f;
const float WORLD_BASE_Y   =  0.0000f;
const float WORLD_BASE_Z   =  0.0000f;
const float WORLD_BASE_YAW =  0.0021f;   // rad

enum { J_BASE = 0, J_SHOULDER, J_ELBOW, J_GRIPPER, N_J };
const char *JNAME[N_J] = {"base", "shoulder", "elbow", "gripper"};

// JointCalibration:  q_sim = SIGN * q_interno + OFFSET
const float CAL_SIGN[N_J]   = {-1.0f, 1.0f, -1.0f, 1.0f};
const float CAL_OFFSET[N_J] = {0.0f, -1.8819885f, -0.0729548f, 0.0f};

// JOINT_LIMITS (rad, en valores "sim")
const float LIM_LO[N_J] = {-3.1415f, -1.550f, -0.275f, 0.000f};
const float LIM_HI[N_J] = { 3.1415f,  0.356f,  0.705f, 1.450f};

const float GRIPPER_OPEN   = 0.8f;
const float GRIPPER_CLOSED = 0.0f;

// ============================================================================
//  MAPEO A SERVOS REALES   <<< TUNE >>>
// ============================================================================
const int SERVO_PIN[N_J] = {13, 14, 27, 26};

// Pulso (us) cuando el joint "sim" vale 0 rad.
float SERVO_US_AT_ZERO[N_J] = {1500.0f, 1500.0f, 1500.0f, 1500.0f};   // TUNE
// +1 / -1 segun el sentido de giro del servo respecto a la sim.
float SERVO_DIR[N_J]        = {1.0f, 1.0f, 1.0f, 1.0f};               // TUNE
// us por radian. Servo de 180 grados con 500..2500 us -> 2000/pi = 636.6
float SERVO_US_PER_RAD[N_J] = {636.62f, 636.62f, 636.62f, 636.62f};   // TUNE

const int SERVO_US_MIN = 500;
const int SERVO_US_MAX = 2500;

// ============================================================================
//  PERFIL DE MOVIMIENTO  (valores para el brazo REAL, no los de la sim)
// ============================================================================
float linVel = 0.10f;   // m/s     (VEL)
float linAcc = 0.30f;   // m/s^2   (ACC)

// Limites por articulacion (rad/s, rad/s^2). MG996R ~ 5 rad/s sin carga.
const float MAX_JOINT_VEL[N_J] = {3.0f, 3.0f, 3.0f, 6.0f};
const float MAX_JOINT_ACC[N_J] = {12.0f, 12.0f, 12.0f, 20.0f};

const float UPDATE_HZ         = 50.0f;   // = refresco del servo
const float MIN_MOVE_DURATION = 0.05f;   // s
const float GRIPPER_MOVE_TIME = 0.40f;   // s, si solo se mueve el gripper
const int   N_CHECK           = 50;      // puntos de verificacion sobre la recta

// Pose de reposo (mundo, m). Es el "at_object" de push_Middle.json.
const float HOME_X = 0.14f, HOME_Y = 0.0f, HOME_Z = 0.1425f;

// Constantes del perfil quintico s = 10t^3 - 15t^4 + 6t^5
const float PEAK_VEL_COEFF   = 1.875f;
const float PEAK_ACCEL_COEFF = 5.7735027f;   // 10/sqrt(3)

// ============================================================================
//  TIPOS Y ESTADO
// ============================================================================
struct Vec3 { float x, y, z; };

struct Move {
  Vec3  p0, p1;
  float g0, g1;     // gripper (rad sim)
  float T;          // duracion (s)
};

const int QUEUE_LEN = 16;
Move queueBuf[QUEUE_LEN];
int  qHead = 0, qCount = 0;

Move     active;
bool     moving = false;
uint32_t moveT0 = 0;

Vec3  curPos;          // posicion cartesiana actual (mundo)
float curGrip;         // gripper actual (rad sim)
Vec3  tailPos;         // donde termina el brazo cuando se vacie la cola
float tailGrip;
bool  poseValid = false;   // false tras RAW/SIM: posicion cartesiana desconocida

float lastCmd[N_J] = {0};
int   lastUs[N_J]  = {0};

Servo servos[N_J];
uint32_t lastTickUs = 0;

char lineBuf[96];
int  lineLen = 0;

// ============================================================================
//  CINEMATICA
// ============================================================================
static inline float quinticS(float tau) {
  return tau * tau * tau * (10.0f + tau * (-15.0f + 6.0f * tau));
}

Vec3 worldToLocal(const Vec3 &w) {
  float dx = w.x - WORLD_BASE_X, dy = w.y - WORLD_BASE_Y, dz = w.z - WORLD_BASE_Z;
  float c = cosf(-WORLD_BASE_YAW), s = sinf(-WORLD_BASE_YAW);
  return { dx * c - dy * s, dx * s + dy * c, dz };
}

float cmdToUs(int j, float q) {
  return SERVO_US_AT_ZERO[j] + SERVO_DIR[j] * SERVO_US_PER_RAD[j] * q;
}

float usToCmd(int j, float us) {
  return (us - SERVO_US_AT_ZERO[j]) / (SERVO_DIR[j] * SERVO_US_PER_RAD[j]);
}

// Grados del servo (0..180) <-> pulso, asumiendo 500us = 0 deg, 2500us = 180 deg
const float SERVO_DEG_SPAN = 180.0f;
float usToDeg(float us) { return (us - SERVO_US_MIN) * SERVO_DEG_SPAN / (SERVO_US_MAX - SERVO_US_MIN); }
float degToUs(float d)  { return SERVO_US_MIN + d * (SERVO_US_MAX - SERVO_US_MIN) / SERVO_DEG_SPAN; }

// (x,y,z) mundo + gripper -> comandos "sim" (rad). Devuelve false y llena err
// si el punto no es alcanzable o algun joint/servo queda fuera de rango.
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

  float c = (d * d - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  c = constrain(c, -1.0f, 1.0f);
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
      snprintf(err, n, "servo %s saturado (%.0f us) en (%.3f,%.3f,%.3f)",
               JNAME[j], us, w.x, w.y, w.z);
      return false;
    }
    cmd[j] = q;
  }
  return true;
}

void writeServos(const float cmd[N_J]) {
  for (int j = 0; j < N_J; j++) {
    int us = (int)lroundf(cmdToUs(j, cmd[j]));
    us = constrain(us, SERVO_US_MIN, SERVO_US_MAX);
    servos[j].writeMicroseconds(us);
    lastCmd[j] = cmd[j];
    lastUs[j]  = us;
  }
}

// ============================================================================
//  PLANIFICADOR LINEAL (port de LinearCartesianPlanner)
// ============================================================================
static inline Vec3 lerp3(const Vec3 &a, const Vec3 &b, float s) {
  return { a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s };
}

bool planMove(const Vec3 &p0, float g0, const Vec3 &p1, float g1, Move &m,
              char *err, size_t n) {
  float dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);

  // 1) Duracion por limites lineales
  float T = 0.0f;
  if (dist > 1e-6f) {
    float tv = PEAK_VEL_COEFF * dist / linVel;
    float ta = sqrtf(PEAK_ACCEL_COEFF * dist / linAcc);
    T = max(tv, ta);
  }
  if (fabsf(g1 - g0) > 1e-6f) T = max(T, GRIPPER_MOVE_TIME);
  T = max(T, MIN_MOVE_DURATION);

  // 2) Recorrer la recta: alcanzabilidad + vel/acc articulares
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

  // 3) Estirar la duracion si alguna articulacion no llega
  float scale = 1.0f;
  for (int j = 0; j < N_J; j++) {
    if (maxV[j] > MAX_JOINT_VEL[j]) scale = max(scale, maxV[j] / MAX_JOINT_VEL[j]);
    if (maxA[j] > MAX_JOINT_ACC[j]) scale = max(scale, sqrtf(maxA[j] / MAX_JOINT_ACC[j]));
  }

  m.p0 = p0; m.p1 = p1; m.g0 = g0; m.g1 = g1; m.T = T * scale;
  return true;
}

bool enqueue(const Vec3 &target, float grip) {
  if (!poseValid) { Serial.println("ERR pose desconocida: mandar HOME primero"); return false; }
  if (qCount >= QUEUE_LEN) { Serial.println("ERR cola llena"); return false; }

  Move m; char err[112];
  if (!planMove(tailPos, tailGrip, target, grip, m, err, sizeof err)) {
    Serial.printf("ERR %s\n", err);
    return false;
  }
  queueBuf[(qHead + qCount) % QUEUE_LEN] = m;
  qCount++;
  tailPos = target; tailGrip = grip;
  Serial.printf("OK -> (%.4f, %.4f, %.4f) grip=%.2f  T=%.2fs  [cola %d]\n",
                target.x, target.y, target.z, grip, m.T, qCount);
  return true;
}

void stopAll() {
  moving = false;
  qCount = 0;
  tailPos = curPos; tailGrip = curGrip;
}

// Avanza la trayectoria; se llama a UPDATE_HZ
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

  Vec3  p = lerp3(active.p0, active.p1, s);
  float g = active.g0 + (active.g1 - active.g0) * s;

  float cmd[N_J]; char err[112];
  if (!solveIK(p, g, cmd, err, sizeof err)) {   // no deberia pasar (ya validado)
    Serial.printf("ERR en movimiento: %s -> STOP\n", err);
    stopAll();
    return;
  }
  writeServos(cmd);
  curPos = p; curGrip = g;

  if (done) {
    moving = false;
    Serial.println("DONE");
    if (qCount == 0) Serial.println("IDLE");
  }
}

// ============================================================================
//  COMANDOS
// ============================================================================
void printHelp() {
  Serial.println(F(
    "\n=== EEZYbotARM Mk2 - control cartesiano lineal ===\n"
    " x y z        mover en recta a (x,y,z) [m, marco mundo]\n"
    " x y          mover en recta manteniendo la altura actual\n"
    " MOVE x y z   idem\n"
    " HOME         ir a home\n"
    " OPEN | CLOSE gripper\n"
    " VEL v        vel. lineal [m/s]     ACC a   acel. lineal [m/s^2]\n"
    " STOP         frenar y vaciar cola\n"
    " POS          estado actual\n"
    " RANGE [z]    rango de x alcanzable (y=0) a la altura z (default: actual)\n"
    " --- control directo de motores (j: 0=base 1=hombro 2=codo 3=gripper) ---\n"
    " DEG j g      servo j a g grados (0..180)\n"
    " DEGS b s e g los 4 servos en grados\n"
    " JOG j d      mover servo j d grados relativo (ej: JOG 1 5)\n"
    " RAW j us     pulso crudo al servo j\n"
    " SIM b s e g  joints 'sim' directos [rad], como motor1..4 en CoppeliaSim\n"
    " --- calibracion ---\n"
    " CAL j q      'el joint j esta AHORA en q rad de la sim' -> fija el cero\n"
    " DIR j        invertir sentido del servo j\n"
    " CALSHOW      imprimir calibracion para pegar en el codigo\n"));
}

// Escribe un pulso directo a un servo (control articular, sin cinematica)
void writeRawUs(int j, float us) {
  int u = constrain((int)lroundf(us), SERVO_US_MIN, SERVO_US_MAX);
  if (moving || qCount) stopAll();
  servos[j].writeMicroseconds(u);
  lastUs[j]  = u;
  lastCmd[j] = usToCmd(j, u);
  poseValid  = false;
}

void printCal() {
  Serial.printf("float SERVO_US_AT_ZERO[N_J] = {%.1ff, %.1ff, %.1ff, %.1ff};\n",
                SERVO_US_AT_ZERO[0], SERVO_US_AT_ZERO[1], SERVO_US_AT_ZERO[2], SERVO_US_AT_ZERO[3]);
  Serial.printf("float SERVO_DIR[N_J]        = {%.1ff, %.1ff, %.1ff, %.1ff};\n",
                SERVO_DIR[0], SERVO_DIR[1], SERVO_DIR[2], SERVO_DIR[3]);
}

void printRange(float z) {
  float xmin = NAN, xmax = NAN, q[N_J]; char e[112];
  for (int i = 0; i <= 400; i++) {
    float x = i * 0.001f;
    if (solveIK({x, 0.0f, z}, GRIPPER_OPEN, q, e, sizeof e)) {
      if (isnan(xmin)) xmin = x;
      xmax = x;
    }
  }
  if (isnan(xmin)) Serial.printf("RANGE z=%.4f: nada alcanzable con y=0\n", z);
  else Serial.printf("RANGE z=%.4f: x de %.3f a %.3f m (y=0; con y!=0 el radio es el mismo)\n",
                     z, xmin, xmax);
}

void printPos() {
  Serial.printf("POS %s xyz=(%.4f, %.4f, %.4f) grip=%.2f %s\n",
                poseValid ? "" : "(DESCONOCIDA)", curPos.x, curPos.y, curPos.z,
                curGrip, moving ? "[moviendo]" : "");
  for (int j = 0; j < N_J; j++)
    Serial.printf("   [%d] %-8s q_sim=%+.4f rad  servo=%6.1f deg  us=%d\n",
                  j, JNAME[j], lastCmd[j], usToDeg(lastUs[j]), lastUs[j]);
  Serial.printf("   vel=%.3f m/s acc=%.3f m/s^2 cola=%d\n", linVel, linAcc, qCount);
}

void goHome() {
  Vec3 h = {HOME_X, HOME_Y, HOME_Z};
  if (poseValid) { enqueue(h, tailGrip); return; }

  // Pose desconocida: escribir directo (el servo salta a la posicion)
  float cmd[N_J]; char err[112];
  if (!solveIK(h, GRIPPER_OPEN, cmd, err, sizeof err)) {
    Serial.printf("ERR HOME invalido: %s\n", err);
    return;
  }
  stopAll();
  writeServos(cmd);
  curPos = tailPos = h;
  curGrip = tailGrip = GRIPPER_OPEN;
  poseValid = true;
  Serial.println("OK HOME (directo)");
}

bool isNumberToken(const char *t) {
  return t && (isdigit((unsigned char)t[0]) || t[0] == '-' || t[0] == '+' || t[0] == '.');
}

void handleLine(char *line) {
  for (char *c = line; *c; c++) if (*c == ',' || *c == ';' || *c == '\t') *c = ' ';

  char *tok[6]; int nt = 0;
  for (char *t = strtok(line, " "); t && nt < 6; t = strtok(NULL, " ")) tok[nt++] = t;
  if (nt == 0) return;

  // Linea que empieza con numero = MOVE implicito
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
    if (end == tok[argStart + i]) { Serial.printf("ERR argumento invalido '%s'\n", tok[argStart + i]); return; }
  }

  if (!strcmp(cmd, "MOVE")) {
    if (na == 3)      enqueue({a[0], a[1], a[2]}, tailGrip);
    else if (na == 2) enqueue({a[0], a[1], tailPos.z}, tailGrip);   // misma altura
    else Serial.println("ERR uso: x y z  |  x y");
  }
  else if (!strcmp(cmd, "HOME"))  goHome();
  else if (!strcmp(cmd, "OPEN"))  enqueue(tailPos, GRIPPER_OPEN);
  else if (!strcmp(cmd, "CLOSE")) enqueue(tailPos, GRIPPER_CLOSED);
  else if (!strcmp(cmd, "STOP"))  { stopAll(); Serial.println("OK STOP"); printPos(); }
  else if (!strcmp(cmd, "POS"))   printPos();
  else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "VEL") && na == 1 && a[0] > 0.0f && a[0] <= 1.0f) {
    linVel = a[0]; Serial.printf("OK VEL %.3f m/s (aplica a movimientos nuevos)\n", linVel);
  }
  else if (!strcmp(cmd, "ACC") && na == 1 && a[0] > 0.0f && a[0] <= 5.0f) {
    linAcc = a[0]; Serial.printf("OK ACC %.3f m/s^2 (aplica a movimientos nuevos)\n", linAcc);
  }
  else if (!strcmp(cmd, "SIM") && na == 4) {
    float q[N_J] = {a[0], a[1], a[2], a[3]};
    for (int j = 0; j < N_J; j++) q[j] = constrain(q[j], LIM_LO[j], LIM_HI[j]);
    stopAll(); writeServos(q); poseValid = false;
    Serial.println("OK SIM (pose cartesiana desconocida -> HOME para volver a mover)");
    printPos();
  }
  else if (!strcmp(cmd, "RAW") && na == 2 && a[0] >= 0 && a[0] < N_J) {
    int j = (int)a[0];
    writeRawUs(j, a[1]);
    Serial.printf("OK RAW %s = %d us = %.1f deg (q_sim equivalente = %+.4f rad)\n",
                  JNAME[j], lastUs[j], usToDeg(lastUs[j]), lastCmd[j]);
  }
  else if (!strcmp(cmd, "DEG") && na == 2 && a[0] >= 0 && a[0] < N_J) {
    int j = (int)a[0];
    writeRawUs(j, degToUs(constrain(a[1], 0.0f, SERVO_DEG_SPAN)));
    Serial.printf("OK DEG %s = %.1f deg (%d us, q_sim = %+.4f rad)\n",
                  JNAME[j], usToDeg(lastUs[j]), lastUs[j], lastCmd[j]);
  }
  else if (!strcmp(cmd, "DEGS") && na == 4) {
    for (int j = 0; j < N_J; j++) writeRawUs(j, degToUs(constrain(a[j], 0.0f, SERVO_DEG_SPAN)));
    Serial.println("OK DEGS");
    printPos();
  }
  else if (!strcmp(cmd, "JOG") && na == 2 && a[0] >= 0 && a[0] < N_J) {
    int j = (int)a[0];
    float d = constrain(usToDeg(lastUs[j]) + a[1], 0.0f, SERVO_DEG_SPAN);
    writeRawUs(j, degToUs(d));
    Serial.printf("OK JOG %s -> %.1f deg (%d us, q_sim = %+.4f rad)\n",
                  JNAME[j], usToDeg(lastUs[j]), lastUs[j], lastCmd[j]);
  }
  else if (!strcmp(cmd, "RANGE") && na <= 1) {
    printRange(na == 1 ? a[0] : tailPos.z);
  }
  else if (!strcmp(cmd, "CAL") && na == 2 && a[0] >= 0 && a[0] < N_J) {
    int j = (int)a[0];
    SERVO_US_AT_ZERO[j] = lastUs[j] - SERVO_DIR[j] * SERVO_US_PER_RAD[j] * a[1];
    lastCmd[j] = a[1];
    Serial.printf("OK CAL %s: %d us <-> %+.4f rad  => SERVO_US_AT_ZERO=%.1f (solo en RAM)\n",
                  JNAME[j], lastUs[j], a[1], SERVO_US_AT_ZERO[j]);
    printCal();
  }
  else if (!strcmp(cmd, "DIR") && na == 1 && a[0] >= 0 && a[0] < N_J) {
    int j = (int)a[0];
    float q = lastCmd[j];                       // mantener el cero en el pulso actual
    SERVO_DIR[j] = -SERVO_DIR[j];
    SERVO_US_AT_ZERO[j] = lastUs[j] - SERVO_DIR[j] * SERVO_US_PER_RAD[j] * q;
    Serial.printf("OK DIR %s = %+.0f (solo en RAM; si ya hiciste CAL, repetilo)\n",
                  JNAME[j], SERVO_DIR[j]);
    printCal();
  }
  else if (!strcmp(cmd, "CALSHOW")) printCal();
  else Serial.println("ERR comando desconocido / argumentos invalidos (HELP)");
}

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

  goHome();          // salta a HOME (no hay realimentacion de posicion)
  printHelp();
  printPos();
  lastTickUs = micros();
}

void loop() {
  readSerial();
  uint32_t now = micros();
  if (now - lastTickUs >= (uint32_t)(1e6f / UPDATE_HZ)) {
    lastTickUs = now;
    motionTick();
  }
}
