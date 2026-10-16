# Document Architecture: AMR Control System UX/UI Specification

## 1. System Overview & User Flows

```
                          [ Home Screen ]
                                 │
            ┌────────────────────┴────────────────────┐
            ▼                                         ▼
   [ General Mode ]                           [ Developer Mode ]
            │                                         │
    ┌───────┴───────┐                    ┌────────────┼────────────┐
    ▼               ▼                    ▼            ▼            ▼
[ Auto Mode ]  [ Manual Mode ]      [ Mapping ]  [ Navigation ]  [ Settings ]
    │               │                    │                         │
    ▼               ▼                    ▼                         ├─ General
Select Target   External Joystick    External Control              └─ Parameter
 Destination      Control            (Map Creation)
```

---

## 2. Information Architecture & Screen Breakdown

### Screen 1: Home Screen (Mode Selection)
* **Objective:** Direct operators to the appropriate operational layer.
* **Layout Components:**
  * **Header:** System Status (Battery %, Connectivity, Emergency Stop Indicator).
  * **Primary Actions:**
    1. **General Operations Card:** Standard user access.
    2. **Developer Mode Card:** Secured access (triggers Passcode Modal).

#### Passcode Authentication Modal (Dev Mode Entry)
* **UI Pattern:** Modal popup overlay.
* **Components:**
  * PIN Input Field (4 to 6 digits).
  * On-screen Numeric Keypad (touchscreen-friendly).
  * Verification/Cancel Buttons.

---

### Screen 2: General Operations Mode

#### Option 2.1: Auto Mode
* **Objective:** Send the AMR to predefined locations safely and efficiently.
* **Layout Components:**
  * **Map View Context:** 2D Top-Down grid displaying current robot position.
  * **Destination Selector List / Grid:**
    * Buttons representing preset targets (e.g., `Station A`, `Warehouse Bay 3`, `Charging Dock`).
  * **Action Controls:**
    * `START MISSION` (Primary green button).
    * `PAUSE / RESUME`.
    * `CANCEL / RETURN TO BASE`.
  * **Status Panel:** Target name, estimated arrival time (ETA), and path progress.

#### Option 2.2: Manual Mode
* **Objective:** Notify the user that manual override via external remote is engaged.
* **Layout Components:**
  * **Status Indicator:** Displays "External Controller Connected / Disconnected".
  * **Telemetry Feedback:** Live linear and angular velocity gauges ($v$, $\omega$).
  * **Safety Overrides:** Large software Emergency Stop button on screen.
  * **Visual Feedback:** Directional vector display showing current remote input.

---

### Screen 3: Developer Mode

#### Option 3.1: Mapping Mode
* **Objective:** Build a SLAM map using an external joystick remote control.
* **Layout Components:**
  * **Live Occupancy Grid / Point Cloud Canvas:** Real-time map rendering.
  * **External Control Status Banner:** Reminds operator to use the hardware joystick.
  * **Mapping Controls:**
    * `START MAPPING`.
    * `SAVE MAP` (Prompts map naming).
    * `DISCARD / RESET`.
  * **Sensor Diagnostics:** LiDAR status, Odometry metrics.

#### Option 3.2: Navigation Testing Mode
* **Objective:** Perform low-level navigation testing, trajectory evaluation, and waypoint debugging.
* **Layout Components:**
  * **Interactive Map Display:** Ability to set 2D Nav Goals manually on screen.
  * **Log Output / Terminal Overlay:** Real-time ROS node/topic diagnostics.
  * **Control Panel:** Planner toggles (Global/Local Planner selection).

#### Option 3.3: Settings Mode
* **Sub-Section A: General Settings**
  * System Time, Language Selection, Display Brightness.
  * Network Configurations (Wi-Fi, IP Settings, ROS Master URI).
  * Calibration Tools (IMU Zeroing, Wheel Calibration).
* **Sub-Section B: Parameter Settings**
  * Navigation Parameters (Max Linear Speed $v_{max}$, Max Angular Velocity $\omega_{max}$, Safety Radius $r$).
  * Costmap Inflaton Radius & Obstacle Tolerances.
  * PID Controller Gain Tuning ($K_p, K_i, K_d$).

---

## 3. UI/UX Style & Design Principles

* **Design Language:** Industrial Touch-First Interface (Optimized for 7"–10" touchscreens).
* **Color Palette:**
  * **Primary Context:** Dark Theme (Reduces display power consumption & glare in industrial settings).
  * **Safety / Action Colors:**
    * Green: Operational / Safe / Ready.
    * Amber / Yellow: Warning / Manual Override.
    * Red: Emergency Stop / Fault State.
* **Typography & Sizing:** Large touch targets (Minimum $48 \times 48\text{ px}$ clickable area).
