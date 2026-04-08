# Robot Picker System Setup Guide

## System Architecture

### **Two Main Components:**

1. **esp.ino** - ESP32 Camera & Detection
   - Runs on ESP32 with camera
   - Uses Edge Impulse ML model for item detection
   - Sends detected items via WebSocket on port 81
   - Serves detection results to frontend

2. **car.ino** - Arduino Robot Control
   - Controls servo motors and arm
   - Runs WebSocket server on port 81
   - Receives item names and executes picking sequence
   - Uses ESP32 GPIO pins for motor/servo control

3. **picker.html** - Control Interface
   - Shows camera detection from esp.ino
   - Lets user select items
   - Sends item name to car.ino
   - Executes automated picking sequence

---

## Hardware Setup

### **car.ino Pin Configuration (ESP32 GPIO):**
```
Motor Pins (L298N):
- ENA = GPIO 25  (Motor A PWM)
- IN1 = GPIO 26  (Motor A Direction 1)
- IN2 = GPIO 27  (Motor A Direction 2)
- ENB = GPIO 32  (Motor B PWM)
- IN3 = GPIO 33  (Motor B Direction 1)
- IN4 = GPIO 34  (Motor B Direction 2)

Servo Pins:
- PIN_BASE  = GPIO 4   (Base rotation servo)
- PIN_FORW  = GPIO 16  (Forward/Backward servo)
- PIN_VERT  = GPIO 17  (Vertical movement servo)
- PIN_CLAW  = GPIO 5   (Gripper/Claw servo)

Sensor Pins:
- SENSOR_L  = GPIO 3   (Left line sensor)
- SENSOR_R  = GPIO A5  (Right line sensor)
```

---

## Installation Steps

### **1. Install Libraries**

In Arduino IDE → Sketch → Include Library → Manage Libraries:
- **ESP32Servo** (by Kevin Harrington)
- **WebSocketsServer** (by Markus Sattler)
- **WiFi** (built-in for ESP32)

### **2. Update WiFi Credentials**

In both `esp.ino` and `car.ino`, update:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### **3. Upload Sketches**

- Upload `esp.ino` to ESP32 with camera
- Upload `car.ino` to Arduino/another ESP32 for robot control

### **4. Get IP Addresses**

Check Serial Monitor for IP addresses:
- ESP32 Camera IP: `192.168.x.x` (or similar)
- Robot Arduino IP: `192.168.x.x`

---

## Usage

### **Accessing the Control Panel**

**From Camera (esp.ino):**
```
http://192.168.1.xxx/  (view detected items)
```

**From Robot Controller (car.ino):**
```
http://192.168.1.yyy/picker.html  (pick items)
```

### **Workflow**

1. **Open camera IP in browser** → See detected items from camera ML model
2. **Open picker.html on robot IP** → Control panel appears
3. **Select an item** by clicking button or typing name
4. **Click "PICK ITEM"** → Robot automatically:
   - Opens claw (500ms)
   - Moves forward (2s)
   - Closes claw & grabs (500ms)
   - Moves backward (1.5s)
   - Returns to default position (500ms)
   - Opens claw & releases (500ms)

---

## Picking Sequence Timeline

```
0-500ms        : Open claw
500-2500ms     : Move arm forward  
2500-3000ms    : Close claw (grab item)
3000-5000ms    : Move arm backward (with item)
5000-5500ms    : Return to vertical default
5500-6000ms    : Open claw (release item)
6000ms+        : Done - ready for next pick
```

---

## Servo Positions

- **Closed (Grab):** 0°
- **Neutral:** 90°
- **Open (Release):** 180°

Adjust servo positions in `executePickingSequence()` if needed.

---

## Troubleshooting

### **car.ino not connecting to WiFi**
- Check SSID/password are correct
- Verify ESP32 board definition is `ESP32 Dev Module`
- Check Serial Monitor (set to 115200 baud)

### **Servos not responding**
- Verify GPIO pins are correct
- Check servo power supply (usually needs separate 5V)
- Test with simple servo sweep code first

### **WebSocket connection failed**
- Ensure both devices are on same WiFi network
- Check IP addresses match (use actual IP, not `localhost`)
- Verify port 81 is not blocked by firewall

### **Item not picked correctly**
- Adjust claw position (0° might need to be different)
- Increase grab time (change 3000ms timing)
- Check servo speed/timing in `executePickingSequence()`

---

## Advanced Customization

### **Change Picking Speed**
Edit timings in `executePickingSequence()`:
```cpp
else if (elapsed < 2500) {  // Change 2500 for different speed
    forPos = 45;    // Move forward
}
```

### **Add More Quick-Select Items**
Edit `picker.html`:
```html
<button class="item-btn" onclick="selectItem(this, 'Pen')">✏️ Pen</button>
```

### **Change Servo Range**
Edit servo min/max in `writeAllServos()` or servo attach calls.

---

**System Ready! 🤖**
