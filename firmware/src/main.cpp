#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

// --- Config (overridable via build_flags) ---
#ifndef LIDAR_BAUD
#define LIDAR_BAUD 115200
#endif
#ifndef ENCODER_PIN_A
#define ENCODER_PIN_A 4  // GPIO4
#endif
#ifndef ENCODER_PIN_B
#define ENCODER_PIN_B 5  // GPIO5
#endif
#ifndef MOTOR_PWM_PIN
#define MOTOR_PWM_PIN 14  // GPIO14
#endif
#ifndef MOTOR_DIR_PIN
#define MOTOR_DIR_PIN 12  // GPIO12
#endif
#ifndef PPR
#define PPR 400
#endif
#ifndef TCP_PORT
#define TCP_PORT 23
#endif

// --- LiDAR UART ---
// ESP8266 Serial (TX=GPIO1, RX=GPIO3) reads LiDAR
// Use Serial1 (TX=GPIO2) for debug if needed

// --- Encoder state ---
static volatile int32_t encoder_pos = 0;
static volatile uint32_t last_encoder_tick = 0;
static uint32_t prev_encoder_time = 0;
static int32_t prev_encoder_pos = 0;

void ICACHE_RAM_ATTR encoder_isr_a() {
  if (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
  last_encoder_tick = micros();
}

// --- WiFi server ---
WiFiServer tcp_server(TCP_PORT);
WiFiClient client;

// --- Motor control ---
void motor_set(int speed, bool dir) {
  digitalWrite(MOTOR_DIR_PIN, dir);
  analogWrite(MOTOR_PWM_PIN, constrain(speed, 0, 1023));
}

void motor_stop() {
  analogWrite(MOTOR_PWM_PIN, 0);
}

// --- Encoder packet (sent periodically) ---
void send_encoder_packet() {
  if (!client) return;
  uint8_t buf[14];
  buf[0] = 0xAA;
  buf[1] = 0x01;

  uint32_t now = millis();
  int32_t pos;
  noInterrupts();
  pos = encoder_pos;
  interrupts();

  float speed = 0;
  uint32_t dt = now - prev_encoder_time;
  if (dt > 0) {
    speed = ((pos - prev_encoder_pos) * 360.0f) / (PPR * dt / 1000.0f);
  }

  buf[2] = (now >> 24) & 0xFF;
  buf[3] = (now >> 16) & 0xFF;
  buf[4] = (now >> 8) & 0xFF;
  buf[5] = now & 0xFF;

  buf[6] = (pos >> 24) & 0xFF;
  buf[7] = (pos >> 16) & 0xFF;
  buf[8] = (pos >> 8) & 0xFF;
  buf[9] = pos & 0xFF;

  memcpy(buf + 10, &speed, 4);

  prev_encoder_time = now;
  prev_encoder_pos = pos;
  client.write(buf, 14);
}

// --- LiDAR UART read + forward ---
void forward_lidar() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (client) {
      client.write(b);
    }
  }
}

// --- Command parser from PC ---
void handle_commands() {
  if (!client) return;
  while (client.available()) {
    char c = client.read();
    if (c == 'f') {
      motor_set(512, HIGH);
    } else if (c == 'b') {
      motor_set(512, LOW);
    } else if (c == 's') {
      motor_stop();
    } else if (c == '+') {
      // Reset encoder position
      noInterrupts();
      encoder_pos = 0;
      interrupts();
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(LIDAR_BAUD);
  Serial.setDebugOutput(false);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  motor_stop();

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder_isr_a, CHANGE);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial1.begin(115200);
  Serial1.print("\nConnecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial1.print(".");
  }
  Serial1.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());

  tcp_server.begin();
  Serial1.printf("TCP server on port %d\n", TCP_PORT);

  if (MDNS.begin("lidar-scanner")) {
    MDNS.addService("tcp", "lidar", TCP_PORT);
  }
}

// --- Main loop ---
void loop() {
  MDNS.update();

  if (!client || !client.connected()) {
    client = tcp_server.available();
    if (client) {
      Serial1.println("PC connected");
    }
    return;
  }

  forward_lidar();
  handle_commands();
  send_encoder_packet();
  delay(10);  // 100 Hz encoder updates
}
