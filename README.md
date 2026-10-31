# Smart Water Network Leakage Detection System

**An Intelligent, Real-Time Water Leakage Detection and Isolation System Using ESP32, Raspberry Pi 5 and QNX Neutrino RTOS**

## 1. Project Overview

Water leakage in distribution networks results in significant water wastage, reduced distribution efficiency, and infrastructure damage. Conventional water monitoring systems often detect abnormal flow conditions without accurately identifying the location of the leakage.

This project presents an intelligent Smart Water Network Leakage Detection System that monitors water flow at multiple points along a pipeline, identifies potential leakage locations, and automatically isolates the affected pipeline segment using electrically controlled valves.

The system uses three flow sensors and two isolation valves connected to an ESP32 microcontroller. The ESP32 communicates with a Raspberry Pi running QNX Neutrino RTOS through the I2C communication protocol.

The Raspberry Pi processes the sensor readings, executes the leakage detection algorithm, identifies the affected pipeline segment, and commands the ESP32 to close the corresponding isolation valve.

### Key Features

- Real-time monitoring of water flow at three measurement points.
- Detection and localisation of leakage across two pipeline segments.
- Automatic isolation of affected pipeline sections.
- ESP32-based sensor acquisition and valve control.
- Raspberry Pi-based central monitoring and processing.
- QNX Neutrino RTOS implementation using multithreading.
- I2C-based communication between the Raspberry Pi and ESP32.
- Configurable leakage detection tolerance and confirmation cycles.
- Fault detection through communication timeouts and packet validation.
- Manual system reset and valve reopening functionality.

---

## 2. System Architecture

The system consists of two major processing units:

**1. ESP32 – Sensor Acquisition and Actuation Unit**

The ESP32 collects pulse signals from three water flow sensors, calculates flow rates, maintains the valve states, and communicates the sensor information to the Raspberry Pi through I2C.

**2. Raspberry Pi 5 – Central Monitoring and Control Unit**

The Raspberry Pi runs a C-based application on QNX Neutrino RTOS. It periodically retrieves sensor measurements, analyses flow differences, identifies leakage locations, and sends valve control commands to the ESP32.

### System Block Diagram

```text
             WATER DISTRIBUTION PIPELINE

   PUMP
     |
     v
   [F1] -------- [V1] -------- [F2] -------- [V2] -------- [F3]
     |                            |                           |
     |                            |                           |
     +----------------------------+---------------------------+
                                  |
                           ESP32 MICROCONTROLLER
                                  |
                           I2C COMMUNICATION
                                  |
                                  v
                         RASPBERRY PI 5
                                  |
                         QNX NEUTRINO RTOS
                                  |
                    +-------------+-------------+
                    |                           |
              I2C POLLING THREAD          LEAK MONITOR THREAD
                    |                           |
              Sensor Acquisition          Flow Comparison
                    |                           |
              ESP32 Communication         Leak Localisation
                    |                           |
                    +-------------+-------------+
                                  |
                          VALVE CONTROL COMMAND
                                  |
                                  v
                         ESP32 MICROCONTROLLER
                                  |
                       +----------+----------+
                       |                     |
                      V1                    V2
                       |                     |
                  ISOLATION             ISOLATION
                   VALVE                  VALVE
```

### Pipeline Configuration

```text
Pump
  |
  F1
  |
  V1
  |
  F2
  |
  V2
  |
  F3
  |
Demand
```

The pipeline is divided into two monitoring segments:

| Segment | Flow comparison | Isolation valve |
|---|---|---|
| Segment 1 | F1 vs F2 | V1 |
| Segment 2 | F2 vs F3 | V2 |

**Leak localisation principle:**

- If F1 is significantly greater than F2, a possible leak is identified in Segment 1. V1 is closed.
- If F2 is significantly greater than F3, a possible leak is identified in Segment 2. V2 is closed.

The system selects the isolation valve closest to the identified affected segment.

---

## 3. Hardware Components

| Component | Quantity | Purpose |
|---|---:|---|
| Raspberry Pi 5 | 1 | Central processing and QNX execution |
| ESP32 | 1 | Sensor acquisition and valve control |
| Water flow sensors (YF-S201 style) | 3 | Water flow measurement |
| Electrically controlled solenoid valves | 2 | Pipeline isolation |
| Water pump | 1 | Water circulation |
| Pipeline and connecting accessories | As required | Water distribution setup |
| External valve driver circuits | 2 | Solenoid valve switching |
| Power supply | As required | System power |

### Flow Sensor Arrangement

- F1 measures the upstream flow after the pump.
- F2 measures the flow between V1 and V2.
- F3 measures the downstream flow after V2.

The three measurements allow the system to compare the incoming and outgoing flow across each pipeline segment.

---

## 4. Software and Technologies

| Technology | Application |
|---|---|
| C Programming | Central monitoring and leakage detection |
| Arduino C/C++ | ESP32 firmware |
| QNX Neutrino RTOS 8.0 | Real-time operating system |
| ESP32 | Sensor and actuator interface |
| Raspberry Pi 5 | Central processing platform |
| I2C Protocol | Communication between ESP32 and Raspberry Pi |
| GPIO Interrupts | Flow sensor pulse acquisition |
| POSIX Threads (Pthreads) | Concurrent task execution |
| QNX Devctl API | I2C communication through the QNX device driver |

---

## 5. ESP32 Firmware – Sensor Acquisition and Valve Control

**File:** `esp32_triple_flow_slave.ino`

The ESP32 operates as an I2C slave device with the address `0x32`.

Its responsibilities include:

1. Reading pulse signals from the three flow sensors.
2. Calculating individual flow rates.
3. Maintaining the current states of V1 and V2.
4. Preparing sensor readings into a structured I2C data packet.
5. Receiving valve control commands from the Raspberry Pi.
6. Operating the corresponding valve through its GPIO output.

### 5.1 Flow Measurement

The YF-S201-style flow sensors generate electrical pulses proportional to the volume of water passing through them.

The ESP32 uses hardware interrupts to count these pulses.

Each sensor has an independent interrupt service routine (ISR):

- `onPulse1()` – Counts pulses from F1.
- `onPulse2()` – Counts pulses from F2.
- `onPulse3()` – Counts pulses from F3.

The system uses a 2-second sliding measurement window divided into four 500 ms intervals.

The flow rate is calculated using:

**Flow Rate (mL/min) = (Total Pulses / Window Time in Seconds) × (60000 / Pulses Per Liter)**

The firmware uses approximately 450 pulses per litre as the initial calibration value.

The resulting measurements are stored as 16-bit unsigned integers and transmitted to the Raspberry Pi.

### 5.2 Valve Control

Two electrically controlled valves are connected to the ESP32.

| Valve | GPIO | Function |
|---|---:|---|
| V1 | GPIO 26 | Isolates Segment 1 |
| V2 | GPIO 32 | Isolates Segment 2 |

The firmware maintains the valve states:

- `1` – Open
- `0` – Closed

The ESP32 receives valve commands from the Raspberry Pi and updates the corresponding GPIO output.

### 5.3 I2C Communication

The ESP32 operates at I2C address:

`0x32`

**Master READ – 10-byte packet**

The Raspberry Pi requests the latest sensor readings and valve states.

| Byte | Description |
|---|---|
| 0 | Header (0xA5) |
| 1 | F1 Low Byte |
| 2 | F1 High Byte |
| 3 | F2 Low Byte |
| 4 | F2 High Byte |
| 5 | F3 Low Byte |
| 6 | F3 High Byte |
| 7 | V1 State |
| 8 | V2 State |
| 9 | XOR Checksum |

**Master WRITE – 4-byte packet**

The Raspberry Pi sends commands to operate the valves.

```text
[0x10][Valve ID][Command][Checksum]
```

| Parameter | Meaning |
|---|---|
| 0x10 | Valve command identifier |
| 0 | V1 |
| 1 | V2 |
| 0 | Close valve |
| 1 | Open valve |

The checksum is calculated as:

`0x10 ^ Valve_ID ^ Command ^ 0x5A`

This protocol enables structured communication and basic data integrity verification between the two processing units.

---

## 6. QNX Master Application – Leakage Detection and Control

**File:** `leakmon2.c`

The Raspberry Pi executes a C application on QNX Neutrino RTOS.

The application is responsible for collecting sensor measurements, detecting abnormal flow differences, identifying leakage locations, and controlling the isolation valves.

### 6.1 Multithreaded Architecture

The QNX application uses three main execution threads.

**1. Main Thread**

Responsible for:
- Initialising the application.
- Configuring I2C communication.
- Creating the polling and monitoring threads.
- Handling system signals.
- Managing shutdown and operator reset operations.

**2. I2C Polling Thread**

Responsible for:
- Reading sensor data from the ESP32.
- Validating received packets.
- Updating flow measurements and valve states.
- Sending valve control commands.
- Handling communication errors and retries.

This is the only thread that directly accesses the I2C interface.

**3. Leak Monitoring Thread**

Responsible for:
- Analysing flow measurements.
- Comparing upstream and downstream flow rates.
- Applying leakage tolerance conditions.
- Confirming persistent flow differences.
- Identifying the affected pipeline segment.
- Requesting the appropriate isolation valve closure.

The monitoring thread does not directly access I2C. Instead, it communicates valve requests to the polling thread using shared data structures and condition variables.

### 6.2 Thread Communication

The application uses POSIX synchronization mechanisms:

- **Mutexes:** Protect shared data from simultaneous access.
- **Condition Variables:** Notify threads when new sensor data or valve commands are available.
- **Shared Data Structures:** Maintain flow readings, valve states, and pending commands.

This design separates sensor communication from leakage analysis and valve actuation.

### 6.3 Leakage Detection Algorithm

The system identifies leakage by comparing the flow rates measured at consecutive sensor positions.

For each pipeline segment:

```text
Upstream Flow  = Flow entering the segment
Downstream Flow = Flow leaving the segment

Flow Difference = Upstream Flow - Downstream Flow
```

The allowable flow difference is calculated using:

`Tolerance = Absolute Tolerance + (Percentage Tolerance / 100 × Upstream Flow)`

A possible leak is detected when:

`Upstream Flow - Downstream Flow > Tolerance`

The system also applies a minimum flow condition to avoid detecting leaks when both sensor readings are below the configured minimum flow.

### 6.4 Leakage Confirmation

To reduce false leak detection caused by temporary flow fluctuations, the system requires the abnormal flow difference to persist for a specified number of consecutive monitoring cycles.

The default confirmation requirement is three consecutive cycles.

For example:

```text
Cycle 1: Abnormal flow difference detected
Cycle 2: Abnormal flow difference detected
Cycle 3: Abnormal flow difference detected

Result: Leak confirmed
Action: Close the corresponding isolation valve
```

### 6.5 Automatic Leak Isolation

| Detection condition | Identified location | System response |
|---|---|---|
| F1 > F2 beyond tolerance | Segment 1 | Close V1 |
| F2 > F3 beyond tolerance | Segment 2 | Close V2 |

Once a leak is confirmed, the corresponding segment is latched.

This prevents repeated triggering of the same leakage event until an operator performs a reset.

### 6.6 Communication Fault Handling

The QNX application incorporates several communication reliability mechanisms:

- I2C read retries.
- Packet header validation.
- XOR checksum verification.
- Communication timeout handling.
- Consecutive communication error tracking.
- Stale sensor data detection.

If valid sensor data is not received within the configured stale-data interval, the monitoring thread suspends leakage evaluation and resets its consecutive detection counters.

---

## 7. System Configuration

The QNX application supports command-line configuration.

| Parameter | Option | Default | Description |
|---|---|---:|---|
| I2C device | `-d` | `/dev/i2c1` | I2C device path |
| Slave address | `-a` | `0x32` | ESP32 I2C address |
| Polling period | `-p` | 500 ms | Sensor polling interval |
| I2C timeout | `-o` | 100 ms | Communication timeout |
| Absolute tolerance | `-t` | 200 mL/min | Minimum absolute flow difference |
| Percentage tolerance | `-r` | 5% | Relative flow tolerance |
| Minimum flow | `-m` | 100 mL/min | Minimum flow for leak evaluation |
| Confirmation cycles | `-n` | 3 | Consecutive detections required |
| Stale data timeout | `-s` | 2000 ms | Maximum permitted data age |

### Example Configuration

```bash
./leakmon2 -d /dev/i2c1 -a 0x32 -p 500 -t 200 -r 5 -n 3
```

This configures:
- 500 ms polling interval.
- 200 mL/min absolute tolerance.
- 5% relative tolerance.
- Three consecutive confirmation cycles.

---

## 8. Installation and Execution

### Step 1: ESP32 Firmware Setup

1. Open Arduino IDE.
2. Install the ESP32 board package by Espressif.
3. Select the appropriate ESP32 development board.
4. Open `esp32_triple_flow_slave.ino`.
5. Verify the GPIO assignments and sensor connections.
6. Upload the firmware to the ESP32.

### Step 2: Raspberry Pi and QNX Setup

1. Set up QNX Neutrino RTOS on the Raspberry Pi 5.
2. Ensure the I2C interface and appropriate QNX I2C driver are available.
3. Connect the ESP32 to the Raspberry Pi using I2C.
4. Verify the I2C device path.
5. Transfer `leakmon2.c` to the Raspberry Pi.

### Step 3: Compile the QNX Application

Using the QNX development environment:

```bash
qcc -Vgcc_ntoaarch64le -o leakmon2 leakmon2.c
```

### Step 4: Execute the Application

```bash
./leakmon2 -d /dev/i2c1 -a 0x32 -p 500 -t 200 -r 5 -n 3
```

The application will initialise the monitoring threads and begin collecting flow measurements.

### Step 5: Monitor System Output

The application prints flow readings, valve states, and detected leakage events.

Example output:

```text
leakmon2: /dev/i2c1 ESP32=0x32
period=500ms tol=200 mL/min + 5.0% confirm=3

F1=5.200 F2=4.100 F3=4.000 L/min V1=OPEN V2=OPEN

*** LEAK in segment 1 (F1-F2) - closing V1 ***

V1 CLOSED confirmed
```

*The output above illustrates the intended logging format; it is an example, not a recorded test result.*

---

## 9. Operator Controls

The QNX application supports signal-based operator control.

| Signal | Function |
|---|---|
| SIGINT | Initiates application shutdown |
| SIGTERM | Initiates application shutdown |
| SIGUSR1 | Clears both leakage latches and requests reopening of V1 and V2 |

To reset the system:

```bash
kill -USR1 <PID>
```

To terminate the application:

```bash
kill -SIGINT <PID>
```

The reset operation reopens both valves and clears the software leakage latches.

---

## 10. Project Directory Structure

Recommended GitHub repository structure:

```text
Smart-Water-Network-Leakage-Detection/
│
├── README.md
│
├── ESP32/
│   └── esp32_triple_flow_slave.ino
│
├── QNX/
│   └── leakmon2.c
│
├── Documentation/
│   ├── system_architecture.png
│   ├── hardware_connections.png
│   └── project_diagram.png
│
└── LICENSE
```

---

## 11. Expected System Operation

The intended operating sequence is:

1. Water flows through the pipeline.
2. Three flow sensors continuously measure the water flow.
3. The ESP32 counts sensor pulses and calculates flow rates.
4. The Raspberry Pi periodically retrieves measurements through I2C.
5. QNX processes the incoming measurements using the monitoring thread.
6. The system compares the flow rates across both pipeline segments.
7. Abnormal differences are checked against configured tolerances.
8. Persistent differences trigger a leakage confirmation.
9. The corresponding isolation valve is automatically closed.
10. The operator can reset the system after inspecting the affected segment.

---

## 12. Applications

- Smart water distribution networks.
- Industrial water pipeline monitoring.
- Residential water management systems.
- Industrial process water distribution.
- Water conservation and infrastructure monitoring.
- Automated pipeline fault detection and isolation.

---

## 13. Future Improvements

Potential enhancements include:

- Integration of a real-time monitoring dashboard.
- Remote alerts through IoT connectivity.
- Historical water consumption and leakage analysis.
- Predictive maintenance using machine learning.
- Integration of additional monitoring nodes for larger pipeline networks.
- Advanced fault classification and sensor failure detection.
- Remote system monitoring and operator control.

---

## 14. Conclusion

The Smart Water Network Leakage Detection System demonstrates the integration of embedded systems, real-time operating systems, sensor networks, and automated control.

By combining ESP32-based flow acquisition with Raspberry Pi processing under QNX Neutrino RTOS, the system aims to identify leakage locations and isolate affected pipeline segments through automated valve control.

The architecture separates sensor acquisition, data communication, leakage analysis, and actuation into distinct functional components, providing a foundation for scalable and intelligent water distribution monitoring.

---

**Technologies:** C | Embedded C/C++ | ESP32 | Raspberry Pi 5 | QNX Neutrino RTOS | I2C | IoT | Real-Time Systems

**Project:** Smart Water Network Leakage Detection System
