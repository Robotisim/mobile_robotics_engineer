## 🔧 Step 1: Motors with L298D

### ✅ **Explanation**
You connect the **2 encoder motors** to the **L298D motor driver**. Each motor has two power wires (for spinning the motor) and encoder wires (which we’ll wire later).

- **OUT1/OUT2** → Motor A (Left)
- **OUT3/OUT4** → Motor B (Right)

### 🔌 **ASCII Diagram**
```
     Motor A              Motor B
    +-------+            +-------+
    |       |            |       |
    |   O   |            |   O   |
    |       |            |       |
    +---+---+            +---+---+
        |                    |
     +--+--+              +--+--+
     |OUT1|-------------->|OUT3|
     |OUT2|-------------->|OUT4|
     |    L298D Motor Driver   |
     +-------------------------+
```

---

## 🧱 Step 2: Vero Board to Hold ESP32 and Power Rails

### ✅ **Explanation**
You place the **ESP32 DevKit** onto the **vero board**, allowing you to distribute power (3.3V or 5V) and GND neatly to all components from this central hub.

### 🔌 **ASCII Diagram**
```
+----------------------+
|      Vero Board      |
| +3.3V Rail      GND  |
|   |                 |     ESP32 DevKit v1
|   |                 |     +--------------+
|   |                 |     | 3V3   GND     |
|   |---------------------> | VCC   GND     |
|                         | |              |
+-------------------------+ +--------------+
```

---

## ⚙️ Step 3: L298D Motor Control Wires to ESP32

### ✅ **Explanation**
To control direction and speed of motors, connect the **6 control pins** of L298D to **ESP32** GPIOs:

- IN1, IN2 → Motor A direction
- IN3, IN4 → Motor B direction
- ENA, ENB → PWM speed control

### 🔌 **ASCII Diagram**
```
L298D Control     →     ESP32 GPIO
-----------              ------------
IN1     ---------------> D5
IN2     ---------------> D4
IN3     ---------------> D18
IN4     ---------------> D19
ENA     ---------------> D13 (PWM)
ENB     ---------------> D32 (PWM)
```

---

## 🧭 Step 4: Encoders from Motors to ESP32

### ✅ **Explanation**
Each encoder motor provides **2 encoder outputs**. You connect these to **interrupt-capable pins** on the ESP32 to read rotation and speed.

Example:
- Encoder A → D34, D35
- Encoder B → D32, D39

### 🔌 **ASCII Diagram**
```
Motor A Encoder       →    ESP32
------------------         ------------
Channel A1 ----------->    D14
Channel A2 ----------->    D26

Motor B Encoder
Channel B1 ----------->    D27
Channel B2 ----------->    D25
```

---

## 🔋 Step 5: Encoder Motors and L298D GND/5V to Vero Board (for ESP32)

### ✅ **Explanation**
- Connect the **encoder VCC and GND** to the vero board.
- L298D **5V output** (if it has 5V reg from 12V input) can supply ESP32 via vero board.

⚠️ *Make sure L298D’s 5V regulator is jumper-enabled if you’re powering ESP32 from it.*

### 🔌 **ASCII Diagram**
```
Encoder Motor Power:
   VCC → Vero +5V rail
   GND → Vero GND rail

L298D:
   5V  → Vero +5V rail (optional for ESP32 VCC)
   GND → Vero GND rail

ESP32 (via Vero):
   3.3V or 5V → from Vero power rail
   GND       → from Vero GND
```

---

## 🔋 Step 6: LiPo Battery to Power Switch and L298D

### ✅ **Explanation**
Your 3S battery (11.1V) connects to a **power switch**, which then powers the L298D motor driver. This provides motor voltage and, optionally, regulated 5V for logic (only if your L298D has 5V output).

- Power switch in series on **positive wire**
- Ground goes directly to L298D and Vero board

### 🔌 **Final Form ASCII Diagram**
```
[3x 18650 (3S)]
  +12.6V Max
     |
     | (positive)
     V
+-----------+
| Power Btn |
+-----------+
     |
     V
  +--------+       GND
  | L298D  |<-------------------+
  | VCC → 12V in                |
  +--------+                   |
                               |
                            +--+--+
                            | GND |→ Vero board GND rail
                            +-----+
```

---

              [ 3x 18650 Li-ion Battery (3S) ]
               +12.6V Max Output
                    |
                    | (Positive)
             +-------------------+
             |   Power Switch    |
             +-------------------+
                    |
                    V
           +---------------------+
           |      Vero Board     |<--------------------------+
           |  +12V / +5V / GND   |                           |
           +---+----------+------+                           |
               |          |                                  |
               |          |                                  |
               |          |                                  |
         +-----+--+    +--+-----+                        +----+-----+
         | L298D  |    | ESP32  |                        | Encoder  |
         | Motor  |    | DevKit |                        | Motors   |
         | Driver |    +--------+                        +----------+
         |        |    | GPIO26 <-------- IN1 ------------------+   |
         |        |    | GPIO27 <-------- IN2 ------------------|---+
         |        |    | GPIO14 <-------- IN3 ------------------+   |
         |        |    | GPIO12 <-------- IN4 ------------------|---+
         |        |    | GPIO25 <-------- ENA (PWM)             |
         |        |    | GPIO33 <-------- ENB (PWM)             |
         |        |    | GPIO34 <-------- Encoder A1            |
         |        |    | GPIO35 <-------- Encoder A2            |
         |        |    | GPIO32 <-------- Encoder B1            |
         |        |    | GPIO39 <-------- Encoder B2            |
         |        |    | 3V3 ←----+ Vero Board (+3.3V or 5V)     |
         |        |    | GND ←----+ Vero Board (GND)            |
         |        |    +----------------------------------------+
         |
         |-- OUT1 → Motor A (+)
         |-- OUT2 → Motor A (−)
         |-- OUT3 → Motor B (+)
         |-- OUT4 → Motor B (−)
         |
         |-- 5V  → Vero Board +5V (optional for ESP32)
         |-- GND → Common Ground rail
         +----------------------------------------+

            [ MPU6050 IMU Sensor ]
                   VCC → ESP32 3.3V
                   GND → ESP32 GND
                   SDA → GPIO22 (ESP32)
                   SCL → GPIO23 (ESP32)

            [ 2D LiDAR (e.g., RPLiDAR A1) ]
                   USB →
