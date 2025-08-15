// Velocity Profile Demo (Arduino IDE compatible)
// - Moves along a straight line to (Xf, Yf)
// - Generates trapezoidal or triangular speed profile based on distance
// - Prints CSV: t,speed,vx,vy  (units follow your inputs)

#include <Arduino.h>
#include <math.h>

// ---------- CONFIGURE THESE ----------
const float Xf = 10.0f;
const float Yf = 0.0f;

const float velocity_max = 5.0f;  // max speed (e.g., m/s)
const float acceleration = 2.0f;   // accel/decel magnitude (e.g., m/s^2)

const float dt = 0.05f;            // time step for printing profile (seconds)
// ------------------------------------

struct ProfileParams {
  bool triangular;
  float ux, uy;       // unit direction vector
  float t_acc;        // accel time
  float t_cruise;     // cruise time (0 in triangular)
  float total_time;   // total motion time
  float vmax_eff;     // effective vmax (same as velocity_max if trapezoidal)
};

ProfileParams params;

float speedAtTime(float t) {
  // Piecewise speed function
  if (params.triangular) {
    // accelerate up then down
    if (t < params.t_acc) {
      return acceleration * t;
    } else {
      return acceleration * (params.total_time - t);
    }
  } else {
    // trapezoidal: accel -> cruise -> decel
    if (t < params.t_acc) {
      return acceleration * t;
    } else if (t < params.t_acc + params.t_cruise) {
      return params.vmax_eff;  // equals velocity_max for trapezoid
    } else {
      return acceleration * (params.total_time - t);
    }
  }
}

void computeProfile() {
  // Geometry
  const float dx = Xf;
  const float dy = Yf;
  const float distance_total = sqrtf(dx * dx + dy * dy);

  if (distance_total <= 1e-6f) {
    // No movement case
    params.triangular = true;
    params.ux = 0.0f; params.uy = 0.0f;
    params.t_acc = 0.0f;
    params.t_cruise = 0.0f;
    params.total_time = 0.0f;
    params.vmax_eff = 0.0f;
    return;
  }

  params.ux = dx / distance_total;
  params.uy = dy / distance_total;

  // Candidate accel phase using requested vmax
  float t_acc = velocity_max / acceleration;
  float d_acc = 0.5f * acceleration * t_acc * t_acc;

  if (2.0f * d_acc <= distance_total) {
    // Trapezoidal
    params.triangular = false;
    params.vmax_eff = velocity_max;
    params.t_acc = t_acc;
    params.t_cruise = (distance_total - 2.0f * d_acc) / velocity_max;
    params.total_time = 2.0f * params.t_acc + params.t_cruise;
  } else {
    // Triangular (never reaches commanded vmax)
    params.triangular = true;
    params.vmax_eff = sqrtf(acceleration * distance_total);
    params.t_acc = params.vmax_eff / acceleration;
    params.t_cruise = 0.0f;
    params.total_time = 2.0f * params.t_acc;
  }
}

void printSummary() {
  Serial.println(F("# Velocity profile summary"));
  Serial.print(F("# Type: ")); Serial.println(params.triangular ? F("triangular") : F("trapezoidal"));
  Serial.print(F("# t_acc(s): ")); Serial.println(params.t_acc, 6);
  Serial.print(F("# t_cruise(s): ")); Serial.println(params.t_cruise, 6);
  Serial.print(F("# total_time(s): ")); Serial.println(params.total_time, 6);
  Serial.print(F("# vmax_eff: ")); Serial.println(params.vmax_eff, 6);
  Serial.print(F("# direction(u_x,u_y): ")); Serial.print(params.ux, 6); Serial.print(F(", "));
  Serial.println(params.uy, 6);
  Serial.println(F("t,speed,vx,vy"));
}

void setup() {
  Serial.begin(115200);
  // On boards with native USB, wait for the serial monitor (optional safety).
  // while (!Serial) { }

  computeProfile();

  if (params.total_time <= 0.0f) {
    Serial.println(F("# Zero distance: no motion."));
    return;
  }

  printSummary();

  // Stream the profile at fixed dt
  // (You can replace this loop with real-time control output if desired.)
  int long count = 0;
  for (float t = 0.0f; t <= params.total_time + 1e-6f; t += dt) {
    count++;
    float speed = speedAtTime(t);
    float vx[count] = speed * params.ux;
    float vy[count] = speed * params.uy;

    Serial.print(t, 3);      Serial.print(',');
    Serial.print(speed, 3);  Serial.print(',');
    Serial.print(vx, 3);     Serial.print(',');
    Serial.println(vy, 3);
  }
}

void loop() {
  // Nothing to do repeatedly; all results printed in setup().
}

