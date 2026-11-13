# Bill of materials — ShiftGate Rover

Prices are **approximate**, in Indian rupees, **checked 13 November 2026**.  They will move; treat the links as the source of truth.  Where a site blocked an automated fetch (Robu.in returned HTTP 403 to this authoring environment) the price is taken from the public product listing and marked *listing, bot-blocked*.

**Recommended kit total: ≈ ₹2,400–2,700** before shipping, well under the ₹3,500 target.  Robocraze currently waives shipping above ₹999; grouping the Robocraze lines into one order is the cheap way to buy.

A USB Type-C cable and a 5 V power bank are assumed to be already on the desk (“bring your own”).  If you need to buy them, add roughly ₹80 + ₹300.

## Recommended electronics

| # | Item | Qty | Approx. ₹ | Supplier (verified 13 Nov 2026) | Notes |
|---|---|---|---:|---|---|
| 1 | ESP32 DevKit V1, 30-pin, **USB-C**, ESP32-WROOM-32 | 1 | 380 | [RobuKits](https://robukits.in/product/865/esp32-development-board-esp32-devkit-v1-30-pins-c-type) (HTTP 200) | Cheaper alt: [Circuit City](https://circuit-city.in/product/esp-32/) ₹349, Micro-USB. |
| 2 | N20 6 V **100 RPM with Hall encoder** | 2 | 558 × 2 = **1,116** | [Robu.in](https://robu.in/product/n20-6v-100rpm-micro-metal-gear-motor-with-encoder/) (*listing, bot-blocked*) | 3 PPR, D-shaft 3 mm.  This is the single most important “don’t cheap out” line: encoders make odometry teachable. |
| 3 | DRV8833 dual motor driver | 1 | 72 | [Robocraze](https://robocraze.com/products/drv8833-2-channel-dc-motor-driver) (HTTP 200) | 2.7–10.8 V, 1.2 A/ch — correct for N20, unlike an L298N. |
| 4 | HC-SR04 ultrasonic (2–400 cm) | 1 | 73 | [Robocraze](https://robocraze.com/products/hc-sr-04-ultrasonic-sensor) (HTTP 200) | **5 V echo.**  Use a 1 kΩ / 2 kΩ divider into the ESP32 (see [hardware/wiring.md](hardware/wiring.md)). |
| 5 | IR obstacle module, 2–30 cm, digital | 2 | 32 × 2 = **64** | [Robocraze](https://robocraze.com/products/ir-proximity-sensor-1) (HTTP 200) | *Not* a TCRT5000 line sensor (those only see 1–8 mm).  Set the pot for ≈ one cell (≈ 12–18 cm). |
| 6 | 4×AA battery holder (series, ≈ 6 V) | 1 | 33 | [Robocraze](https://robocraze.com/products/4-aa-battery-holder) (HTTP 200) | 62 × 58 × 15 mm.  Alkaline ≈ 6 V; NiMH ≈ 4.8 V, also fine.  Do **not** use 6×AA (9 V) with these motors. |
| 7 | AA cells, alkaline or NiMH | 4 | ≈ 80 | Local supermarket / hardware shop | Or use cells you already have (₹0 incremental). |
| 8 | Jumper wire set, 120 pcs, 20 cm (40 M-M, 40 M-F, 40 F-F) | 1 | 145 | [Robocraze](https://robocraze.com/products/jumper-wire-set-m2m-m2f-f2f-40-pcs-each) (HTTP 200) | |
| 9 | 170-point mini breadboard | 1 | 18 | [Robocraze](https://robocraze.com/products/mini-breadboard) (HTTP 200) | 45 × 35 mm, sits on the chassis. |
| 10 | 1 kΩ and 2 kΩ resistors (¼ W) | 1 each | ≈ 5 | Local electronics shop, or any resistor kit | Echo-pin divider only. |
| 11 | Mini slide / toggle switch (SPST) | 1 | ≈ 20 | Local electronics shop | Motor power cut.  Approximate; buy locally. |
| 12 | M3×8 and M3×12 screws + nuts, ≈ 20 pcs, and 4× M2 | 1 bag | ≈ 40 | Hardware shop | Approximate.  Motor clamps and the US bracket use M3. |
| 13 | 8 mm steel bicycle ball (or bearing ball) | 1 | ≈ 10 | Cycle shop | Drops into the printed caster cup. |

**Recommended electronics subtotal: ≈ ₹2,050–2,160** (with new AA cells and a new USB-C cable skipped).

## Mechanical / consumables

| # | Item | Qty | Approx. ₹ | Where | Notes |
|---|---|---|---:|---|---|
| 14 | PLA filament, 1.75 mm, any colour + a contrasting spool for the gate | ~250 g typical sliced, 327 g solid-volume upper bound | 175–250 | Any Indian filament vendor (eSun, WOL3D, local) | 1 kg rolls are typically ₹600–800.  Cost is the *used* fraction. |
| 15 | 5 mm foam board or corrugated cardboard, ≈ 70 × 100 cm | 1 sheet | ≈ 80–150 | Stationery shop | Cut into 176 × 50 mm wall strips.  Cheaper than printing every wall. |
| 16 | Wide rubber bands | 2 | 0–10 | Household / stationery | Tyres, stretched into the wheel groove. |

## Optional (nice, not required)

| # | Item | Qty | Approx. ₹ | Supplier | Why |
|---|---|---|---:|---|---|
| A | MPU-6050 6-axis IMU | 1 | 141 | [Robocraze](https://robocraze.com/products/mpu-6050-triple-axis-accelerometer-gyroscope-module) (HTTP 200) | Challenge extension: fuse IMU with wheel odometry.  I²C on GPIO 21/22. |
| B | 400-point half breadboard | 1 | 39 | [Robocraze](https://robocraze.com/products/half-breadboard) (HTTP 200) | Bench prototyping; too large for the chassis. |
| C | USB Type-C cable | 1 | ≈ 80 | Bring your own, or any reputable seller | Needed if you don’t already have one. |
| D | 5 V power bank, ≥ 1 A | 1 | ≈ 300 | Bring your own | Powers the ESP32 cleanly, isolated from motor noise. |
| E | Printed `wall_segment` instead of foam board | as needed | filament only | `cad/stl/wall_segment.stl` | ≈ 30 g and 2 h each.  Only worth it if you have no foam board. |

## Budget variant (no encoders)

Replace line 2 with two [N20 6 V 60 RPM motors without encoders](https://robocraze.com/products/n20-6v-60-rpm-micro-metal-gear-box-dc-motor) at ₹244 each (HTTP 200, ₹488 total).  You save ≈ ₹630, but Module 2’s 20 cm straight-line test becomes much harder and the occupancy map will drift.  Only choose this if you also add the MPU-6050 and accept a weaker odometry grade.  **The recommended kit uses encoders.**

## What you already need (not in the ₹ total)

- A 3D printer with a ≥ 220 mm bed (Ender 3 / Neptune / Prusa Mini class is enough).  Every part fits an Ender 3.
- A computer with USB and, for ROS 2, Ubuntu 22.04 (Humble) or 24.04 (Jazzy).
- A 1 m-class table for the 5×5 arena (the outer frame is ≈ 924 mm).

## Power architecture (don’t skip this)

```
4×AA (≈ 6 V) ── SPST switch ── DRV8833 VM
                                 DRV8833 GND ──┐
USB 5 V (power bank or cable) ── ESP32 USB     ├── common ground
                                 ESP32 3V3 ── sensors, encoders
                                 ESP32 VIN is unused
```

Never power the motors from the ESP32’s 5 V or 3.3 V pin.  Always share a **common ground**.
