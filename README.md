# LDS07RR Lidar Scanner

Turn a **Roborock LDS07RR** laser distance sensor (the spinning lidar from the Roborock S8 / Q7 Max / Q8 Max / P10 generation) into a standalone 360° scanner with an **ESP32**, a handful of cheap parts and some Python — including **live radar view, Wi-Fi streaming, 2D mapping (SLAM) and a first stab at automatic floor plans**.

There was no public documentation for this module when we started. Everything below (pinout, protocol, motor behaviour) was measured and reverse-engineered on a real unit. It is written up in detail so you don't have to repeat the detours.

> ⚠️ Wire colours and behaviour are from **one** module. Always verify with a multimeter before applying power (see [How the pinout was found](#how-the-pinout-was-found)).

---

## Contents

- [What works](#what-works)
- [Hardware](#hardware)
  - [Bill of materials](#bill-of-materials)
  - [LDS07RR pinout](#lds07rr-pinout)
  - [Wiring](#wiring)
- [The protocol](#the-protocol)
- [Firmware (ESP32)](#firmware-esp32)
- [PC software](#pc-software)
- [How the pinout was found](#how-the-pinout-was-found)
- [Lessons learned / pitfalls](#lessons-learned--pitfalls)
- [Limitations and roadmap](#limitations-and-roadmap)
- [Credits](#credits)

---

## What works

| Feature | Status |
|---|---|
| Reading the lidar (115200 baud, Neato-style packets, checksums verified) | ✅ |
| Closed-loop motor speed control on the ESP32 (300 rpm ±2 rpm) | ✅ |
| Standalone operation from a USB power bank (auto-start on power-up) | ✅ |
| Raw data streaming over Wi-Fi (TCP), no packet loss at ~10 kB/s | ✅ |
| Live 360° radar view on the PC | ✅ |
| 2D map building (ICP scan matching + occupancy grid), record & replay | ✅ works indoors, see limitations |
| Automatic floor plan (walls, dimensions, PNG + DXF export) | 🧪 experimental |

Measured on the test unit:

| | Motor straight on 3.3 V | With speed control |
|---|---|---|
| Speed | ~497 rpm | 297–301 rpm |
| Valid samples | ~16 % | 43–59 % (depends on the room) |
| Points per revolution | ~70 | ~150–200 |

---

## Hardware

### Bill of materials

| Part | Notes |
|---|---|
| Roborock **LDS07RR** | Label: *LDS Module, Model LDS07RR, 5 VDC / 2 W, Class 1 laser product*. Sold as a spare part for S8 / S8 Pro Ultra / Q7 Max / Q8 Max / P10. |
| **ESP32 dev board** | Tested with a classic ESP32-D0WD-V3 "DevKit" (30-pin, CH340 USB). Any classic ESP32 works; S2/S3/C3 need other pins. |
| **BC547B** NPN transistor | Low-side motor switch. Max 100 mA; the motor draws ~55 mA. A logic-level MOSFET (AO3400, IRLZ44N…) works too. |
| **200 Ω** resistor | Transistor base resistor (1 kΩ is also fine and gentler on the GPIO). |
| **1000 µF / 25 V** electrolytic capacitor | Across VIN–GND. Prevents ESP32 brown-outs from motor inrush current. |
| Flyback diode 1N4148 / 1N4007 *(recommended)* | Across the motor, cathode (stripe) to +. |
| USB power bank | For walking around. Total draw is roughly 200–300 mA. |
| Jumper wires, multimeter | The module's connector has ~1.25 mm pitch; we back-probed it with thin pins. |

### LDS07RR pinout

The module has **6 wires** in one connector. On our unit the motor wires simply pass through the PCB to the motor (connector J4), so the module does **not** control its own motor.

| Wire colour | Function | Notes |
|---|---|---|
| **orange** | **+5 V** | Module supply. |
| **white** | **GND** | |
| **yellow** | **UART TX** (module → host) | 115200 8N1. Idles ~2.2 V on a multimeter, reads fine on a 3.3 V ESP32 input. |
| **brown** | unknown | Probably RX. Static high/low, PWM and ~60 candidate start commands had no effect. Not needed. |
| **red** | **motor +** | Continuity 0.1 Ω to the motor's red wire. |
| **black** | **motor −** | Continuity 0.1 Ω to the motor's black wire. |

Diode-test matrix (module unpowered, meter in diode mode, `0L` = open) — handy to compare with your own unit:

| Red probe → black probe | Reading |
|---|---|
| white → orange | **0.549 V** (lowest: GND → supply) |
| white → yellow | 0.686 V |
| yellow → white | 0.99 V |
| orange → yellow | 2.345 V |
| brown ↔ orange / yellow / white | 0L both ways |

### Wiring

| From | To |
|---|---|
| LDS orange (+5 V) | ESP32 **VIN** (5 V from USB / power bank) |
| LDS white (GND) | ESP32 **GND** |
| LDS yellow (TX) | ESP32 **GPIO32** |
| LDS brown | not connected |
| Motor red (+) | ESP32 **VIN** |
| Motor black (−) | BC547 **collector** |
| BC547 **emitter** | GND |
| BC547 **base** | 200 Ω → ESP32 **GPIO26** |
| 1000 µF capacitor | + to VIN, − (stripe) to GND |
| Flyback diode (optional) | across the motor, stripe to VIN |

```
ESP32 VIN (5 V) ──┬───────────┬──────────────┬──── LDS orange (+5 V)
                  │           │              │
              motor red   diode (stripe)  1000 µF (+)
                 (M)          │              │
              motor black ────┘              │
                  │                          │
                  C                          │
ESP32 GPIO26 ─[200 Ω]─ B  BC547B             │
                  E                          │
                  │                          │
ESP32 GND ────────┴──────────────────────────┴──── LDS white (GND)

ESP32 GPIO32 ──────────────────────────────────── LDS yellow (TX)
```

BC547 pin order: flat side facing you, legs down → **C, B, E** (left to right).

Why the motor is on VIN and not on 3V3: with the motor on the ESP32's 3.3 V rail, its inrush current browned out the ESP32 (reset reason `BROWNOUT` / power-on). On VIN with the 1000 µF capacitor and a soft start in the firmware, the problem is gone. With PWM the 5 V supply is no issue — the control loop settles at roughly 35–60 % duty.

---

## The protocol

**UART 115200 8N1**, module → host only. No start command is needed: the module talks as soon as it has 5 V.

### While the head is not spinning: status packet (`0xAA`)

About every 500 ms the module sends a status/property packet starting with `0xAA 0x2A 0x00 ...`. It contains the module's serial number in ASCII and the string `G431` (the MCU is most likely an STM32G431). This looks like the "property packet" described in Roborock's own open-source [Cullinan](https://github.com/Roborock-OpenSource/Cullinan) lidar project. We did not decode it further.

### While spinning: measurement packets (`0xFA`)

Once the head turns fast enough, the module streams the classic **Neato XV-11 / Xiaomi LDS02RR** format: 22-byte packets, **90 packets per revolution**, 4 samples each (1° per sample).

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `0xFA` start byte |
| 1 | 1 | index `0xA0`–`0xF9` → first angle = (index − 0xA0) × 4 |
| 2 | 2 | speed, uint16 little-endian, **rpm × 64** |
| 4 | 4 × 4 | 4 samples: `dist_lo`, `dist_hi`, `strength_lo`, `strength_hi` |
| 20 | 2 | checksum, uint16 little-endian |

Per sample: distance in **mm** = `dist_lo | (dist_hi & 0x3F) << 8`. Bit 7 of `dist_hi` = **invalid** (then `dist_lo` holds an error code; we saw `0x11` and `0x20`), bit 6 = signal-strength warning.

Checksum (same as Neato):

```python
def neato_checksum(pkt):          # pkt = the 22 bytes
    chk = 0
    for i in range(0, 20, 2):
        chk = (chk << 1) + (pkt[i] | pkt[i + 1] << 8)
    chk = (chk & 0x7FFF) + (chk >> 15)
    return chk & 0x7FFF           # must equal pkt[20] | pkt[21] << 8
```

### Behaviour worth knowing

- **Speed matters.** The module is designed for ~300 rpm (5 Hz). At ~500 rpm only ~16 % of the samples are valid; at 300 rpm 43–59 %.
- **Stopping kills it.** If the head stops (or slows down a lot) while measuring, the module goes **silent until it is power-cycled**. Keep the motor running, and don't reset the controller mid-session (see [pitfalls](#lessons-learned--pitfalls)).
- **Range** is about 0.12–6 m indoors. Direct sunlight (infrared) reduces the number of valid samples considerably.

---

## Firmware (ESP32)

`lds_probe/lds_probe.ino` started life as a reverse-engineering probe and grew into the scanner firmware. On power-up it **starts by itself**: it reads the lidar on GPIO32, regulates the motor on GPIO26 to 300 rpm, forwards the raw lidar bytes over USB serial (921600 baud), and — if Wi-Fi is configured — over **TCP port 2323** (mDNS name `lds`).

### Build and flash

Arduino core for ESP32 (3.x) via `arduino-cli`:

```bash
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 --upload --port COM6 lds_probe
```

(Or open the sketch in the Arduino IDE, board "ESP32 Dev Module".)

### Commands (USB serial, 921600 baud, one per line)

| Command | What it does |
|---|---|
| `scan [ms] [pu]` | Samples all probe pins (GPIO 25, 26, 27, 32) at ~1 µs: idle level, edge count, shortest pulse and a baud-rate estimate. `pu` enables pull-ups. This is how we found the TX line. |
| `bridge <rx> <baud> [tx] [od]` | Raw UART bridge lidar ↔ PC. With `tx` the PC can send bytes too (`od=1` → open-drain TX). |
| `lidar <rx> <baud> <motorpin> <rpm>` | Bridge **plus** motor speed control: parses the `0xFA` packets, reads the rpm and drives a 20 kHz PWM with an integrating controller and soft start. Auto-started at boot as `lidar 32 115200 26 300`. |
| `set <pin> 0\|1\|z` | Drive a pin low / high / release it. |
| `pwm <pin> <freq> <duty%>` | PWM on a pin (resolution adapts, up to MHz). |
| `wifi <ssid> <password>` | Store Wi-Fi credentials in the ESP32's NVS (not in the code) and connect. The SSID may contain spaces, the password may not. 2.4 GHz only. |
| `wifi off` | Forget the credentials. |
| `info` | Uptime, last reset reason (catches brown-outs!), motor duty, Wi-Fi status and IP. |
| `pins` | List the probe pins. |

### Wi-Fi setup (once, over USB)

```bash
py lds.py esp --port COM6 "wifi MyNetwork MyPassword"
py lds.py esp --port COM6 info      # shows the IP address
```

After that you can unplug the PC, power the ESP32 from a power bank and connect over Wi-Fi with `--host <ip>`.

---

## PC software

Python 3.10+ (developed on 3.14, Windows). Install the dependencies:

```bash
pip install -r requirements.txt     # pyserial, numpy, matplotlib, scipy
```

### `lds.py` — lidar toolbox

| Command | Purpose |
|---|---|
| `py lds.py ports` | List serial ports. |
| `py lds.py esp --port COM6 info` | Send a command to the ESP32 (`scan`, `info`, `set 26 1`, …; separate several with `;`). |
| `py lds.py view --port COM6 --esp 32 --motor 26 --rpm 300` | **Live 360° radar** via USB, with motor control. |
| `py lds.py view --host 192.168.x.y` | Same over Wi-Fi. `--range 6000` shows up to 6 m, `--snapshot x.png` saves an image without opening a window. |
| `py lds.py raw --host 192.168.x.y --seconds 10 --out scan.bin` | Record raw bytes. |
| `py lds.py sniff --port COMx` | Reverse-engineering helper: tries 13 baud rates, auto-correlates the stream to find the frame length and recognises known lidar headers (Neato, LD06/LD19, Delta, RPLidar, YDLidar, Camsense). |
| `py lds.py listen --port COMx` | Live bytes/s — move the probe wire until data shows up. |
| `py lds.py poke --port COM6` | Tries a list of known lidar start commands on several baud rates via the ESP32 and listens for an answer. |

Decoders for the Neato format and the LD06 format are included.

### `slam.py` — build a map while walking

```bash
py slam.py --host 192.168.x.y             # live over Wi-Fi (or --port COM6 over USB)
py slam.py --replay recording_XXXX.bin --fast --out map
```

- Assembles one scan per revolution (the first, incomplete revolution is skipped).
- **ICP scan-to-map matching** (point-to-point, trimmed, annealed correspondence distance) estimates position and heading — there is no odometry.
- Keyframes every 15 cm / 8° feed a voxel-filtered reference cloud (KD-tree).
- **Occupancy grid**, 5 cm cells, log-odds, vectorised ray tracing.
- Recovery: after 15 rejected scans in a row it re-anchors at the last known pose.
- Every live session is **recorded automatically** (`recording_<time>.bin`), so maps can be rebuilt offline with different settings. The Wi-Fi client reconnects automatically if the connection drops.
- On exit it writes `map.png` (grid + route) and `map.npz` (grid, resolution, route).

Tips for a good map: hold the lidar level, above furniture (chest height), walk slowly, turn slowly, pass doorways slowly, and end where you started.

### `floorplan.py` — from map to floor plan (experimental)

```bash
py floorplan.py map.npz --out floorplan --occ 1.0 --free -2 --minwall 0.6
py floorplan.py map.npz --manhattan      # force walls to 0°/90° after auto-alignment
```

Extracts walls (RANSAC line fitting at any angle, or axis-aligned runs with `--manhattan`), cleans up the floor area, annotates wall lengths and exports **PNG** and **DXF** (walls on layer `WALLS`, millimetres) for CAD. Dimensions are indicative (±5–10 cm at best).

### `decode_check.py`

Quick statistics for a recording: packets, rpm over time, share of valid samples.

---

## How the pinout was found

This is the part that took the longest — and it is where two AI chatbot answers and one wrong assumption of our own sent us in circles. In short:

1. **Photos of the PCB**: the motor is soldered to J4 on the module's own board ("Wireless Board", the rotating head gets power wirelessly), with 6 wires to the robot.
2. **Continuity**: red/black go straight to the motor (0.1 Ω).
3. **Diode test on all pairs** — do the *complete* matrix. Our meter shows `0L` (open) in a way that looks exactly like "0", which cost us a round.
4. First we powered **yellow** (5 V via a 200 Ω resistor to limit current). The module drew ~1 mA and did nothing. That looked like "sleeping, waits for a command", so we spent a while probing brown/orange with enable levels, PWM up to 500 kHz and dozens of UART start commands (`$`, `0xA5 0x20`, `0xA5 0x60`, sync bytes, ASCII…). Nothing.
5. Completing the diode matrix showed **white → orange = 0.549 V**, the lowest reading: the classic GND → supply signature. With 5 V on **orange**, the module immediately streamed `0xAA` status packets, and with the head spinning, `0xFA` measurement packets. We had been back-powering the MCU through its **TX pin's** protection diode.

Tools that made this fast: the ESP32 `scan` command (all four lines at once, ~1 µs resolution, baud estimate), `raw` captures, and a 200 Ω series resistor on anything we were unsure about.

---

## Lessons learned / pitfalls

- **Don't trust generated pinouts.** Two different chatbot answers gave two different, both wrong, pinouts for this module (one claimed a 4-pin connector with PWM motor control). Measure.
- **~1 mA and "nothing happens" can mean you are powering the chip through a data pin.**
- **Brown-outs:** a small DC motor on the ESP32's 3.3 V rail can reset it. Use VIN, a big capacitor and a soft start. The firmware's `info` shows the last reset reason.
- **Opening a serial port resets most ESP32 boards** (DTR/RTS auto-reset). A reset stops the motor, and the lidar then goes silent until power-cycled. `lds.py` opens the port with DTR/RTS low to avoid this.
- A **25 V 1000 µF** capacitor on a live 5 V rail causes an inrush that can itself brown out the ESP32 — fit it with the power off.
- The lidar needs **~300 rpm**; too fast is just as bad as too slow.

---

## Limitations and roadmap

- **Motion de-skew:** one revolution takes 0.2 s; walking/turning during a scan bends straight walls. Correcting each scan for the motion during the revolution is the next improvement.
- **Loop closure:** drift accumulates; returning to a known place does not pull the map straight yet.
- **Floor plans** are only as good as the map; wall extraction is basic.
- **Outdoors:** sunlight blinds the IR sensor, and open lawns give ICP nothing to lock on to.
- **Lidar power switch:** a MOSFET on the lidar's 5 V would let the firmware power-cycle it automatically when it goes silent.
- **3D:** since the lidar only sees a horizontal slice, recordings at several known heights could be aligned and stacked into a point cloud.

Contributions and measurements from other LDS07RR units are very welcome — especially what the brown wire does and the layout of the `0xAA` status packet.

---

## Credits

- [Roborock-OpenSource/Cullinan](https://github.com/Roborock-OpenSource/Cullinan) — Roborock's documentation of their LDS protocol (status and measurement packets).
- [kaiaai/LDS](https://github.com/kaiaai/LDS) and [kaiaai/awesome-2d-lidars](https://github.com/kaiaai/awesome-2d-lidars) — overview of cheap 2D lidars and the Neato/LDS02RR protocol, motor control approach.
- [Not Black Magic — LiDAR modules](https://notblackmagic.com/bitsnpieces/lidar-modules/) — reverse-engineering write-up of a similar module.

Not affiliated with Roborock. The LDS07RR is a Class 1 laser product; don't open the optical head.

## License

[MIT](LICENSE)
