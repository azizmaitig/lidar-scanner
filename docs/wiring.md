# Wiring

## LiDAR → ESP8266

| LiDAR Pin | ESP8266 Pin |
|-----------|-------------|
| TX        | RX (GPIO3)  |
| GND       | GND         |
| VCC       | External 5V |

baud rates: 115200 .

## Motor + Encoder → H-Bridge → ESP8266

```
ESP GPIO (PWM) ──→ H-Bridge IN1
ESP GPIO (DIR)  ──→ H-Bridge IN2
H-Bridge OUT1 ──→ Motor M+
H-Bridge OUT2 ──→ Motor M-
```

## Encoder → ESP8266

| Encoder Pin | ESP8266 Pin |
|-------------|-------------|
| A (ch A)    | GPIO4 (interrupt-capable) |
| B (ch B)    | GPIO5 (interrupt-capable) |
| G           | GND         |
| V           | 3.3V        |

Encoder channels A/B connect directly to ESP8266 GPIOs with interrupt handlers for quadrature decoding.

## Power

- LiDAR: External 5V supply (ESP's 3.3V regulator insufficient)
- ESP8266: USB or external 3.3V
- H-Bridge: Motor supply voltage (check motor rating)
