# 500W-MRF300-automatic-amplifier

A fully automatic 500 W HF amateur radio amplifier (3.5–28 MHz) using a push-pull MRF300 LDMOS output stage with transmission-line transformers (TLTs). The amplifier automatically measures operating frequency, selects the appropriate low-pass filter, adjusts input drive power, monitors protection systems, and includes an automatic antenna tuner and touchscreen HMI.

**Author:** S56KG (https://www.qrz.com/db/S56KG)
This was a final-year high school project (slovenian "tehnična poklicna matura")

---

## Overview

Commercial automatic HF amplifiers with equivalent specs typically cost **3,000–5,000€**. This project was set out to build a comparable **500W HF amplifier** - with automatic band switching, automatic input leveling (attenuator), and full protection circuitry - for half the price (built for ~1,500€ including six output transistors lost to testing mishaps; a second build would cost roughly **1,300€**).

Unlike most homebrew amplifiers, the goal here was full automation: no manual band switching, no manual tuning sequence - just key up and go.

<img width="3759" height="2040" alt="klub" src="https://github.com/user-attachments/assets/6ab15e26-8592-423f-9afe-7125a805b145" />

## Key Features

- **Fully automatic operation** - no manual warm-up/tuning procedure required
- **Automatic input attenuator** with power measurement, stepping through 0.5–6 dB stages to hit an ideal ~4W drive level
- **Push-pull output stage** using 2× MRF300 LDMOS transistors, up to ~530W PEP, depending on freq.
- **Automatic low-pass filter selection** based on measured carrier frequency (LPF harmonics are ≤ −65 dBc)
- **Dual-MCU architecture** (2× ATmega2560) communicating over UART, with a custom protocol (type-checked, range-checked, heartbeat-monitored)
- **Nextion touchscreen HMI** showing forward/reflected power, SWR, current, temperature, state, band, gain, and losses in real time
- **Full protection**: overcurrent shutdown (soft-latched via SET-RESET logic), SWR protection, thermal protection, bias monitoring
- **Built-in antenna tuner board** (7x7 L-network, based on the N7DDC ATU-100 topology) - hardware complete, control algorithm in testing
- Extra feature connectors for future interfacing (not yet made PC digital control using a PiPico server) 
- Backup analog voltage/current meter on the front panel

## Design Goals

- At least 450W output
- Fully automatic operation without manual tuning
- Comprehensive protection for the MRF300 output devices 
- Support contest-style rapid TX/RX switching
- Maintain harmonic suppression below −65 dBc
- Operate from 3.5–28 MHz with automatic band detection
- Provide a commercial-style touchscreen HMI for all key operational values
- Build using only readily available components on DigiKey

## How It Works


<img width="1030" height="707" alt="block sheme" src="https://github.com/user-attachments/assets/ebda724a-c78e-44e1-b86b-b1060f62fdb8" />

**Transmit state machine:** `INIT → WAIT → FIRST START → INPUT SETUP → LOW POWER TX → TX SWITCH → HIGH POWER TX → READY`. The "ready" state remembers the last transmission and skips most of the setup sequence on the next key-up for faster switching, useful for competition use.


<img width="1764" height="1110" alt="statemachine" src="https://github.com/user-attachments/assets/f326f33c-fa40-4045-8bbf-6fcecfdc51e4" />


## Power/SWR Measurement

Power and SWR are measured using a **tandem match coupler**, rather than a simple directional coupler, since directional couplers perform poorly at low freq. HF. Two transformers (current and voltage) are summed/negated to separate forward and reflected power. Each direction feeds an **AD8310** logarithmic RF power detector (for accuracy across a wide dynamic range, from milliwatts to hundreds of watts), digitized by two 16-bit **ADS1115** ADCs over I2C.

## Boards 
| 1 | Attenuator board | Auto input attenuator (relay-switched, 0.5–6 dB stages, 8 stage), input power/SWR meter, built-in 50ohm dummy load 

| 2 | MCU board | Carrier board for ATmega2560 + ATmega16U2 (Arduino Mega-derived); one per controller 

| 3 | Amplifier board | Push-pull 2× MRF300 output stage, TLT output transformers, high-side current sensing, hardware overcurrent latch 

| 4 | Filter board | Output power/SWR metering + auto-selected low-pass filters (designed in AADE Filter Design + LTspice) 

| 5 | Tuner board | 7x7 L-network automatic antenna tuner (hardware complete, not yet enabled) 

| 6 | Motherboard | MCU carrier, per-board regulated supplies, fan control (6× PWM), 2× ADS1115 ADC, buzzer, overcurrent sensing 

| 7 | Input board | RX/TX switching (opto-isolated relay drive), 50V enable, frequency counter 

All boards are 4-layer FR4 (except the tuner board, 2-layer), designed with trace impedance calculated per JLCPCB dielectric specs.

## Firmware

- Two **ATmega2560** microcontrollers, split because driving the Nextion HMI alone slows a single loop to >30ms. This separates time-critical amplifier control from the touchscreen interface to guarantee deterministic protection response.
  - **Primary MCU**: runs the amplifier state machine, controls the amp stage, PSU, RF switching, attenuator, and filters; measures frequency and enforces fast protection (SWR, current, voltage) every loop.
  - **Secondary MCU**: reads temperature/voltage sensors, drives cooling fans, and manages the Nextion HMI.
- Custom UART protocol between the two MCUs with type checking, range validation, and a heartbeat/ID exchange so a reset on either side is detected.
- Faults are split into **soft** (recoverable, resettable from the HMI) and **hard** (critical) — the system is designed so no single fault destroys hardware (in theory, needs more testing to be sure).
- Developed in PlatformIO / VS Code (migrated from Arduino IDE to allow project specific library configuration, needed to change the UART RX buffer without touching global libraries).
- Combined firmware exceeds 3,000 lines across both MCUs.

## HMI

A **Nextion NX4832F035** touchscreen HMI displays input/output power and SWR (with slider gauges), current, temperature, operating state, faults, detected band, gain, and loss — with on-screen soft-fault reset and SSB mode toggle buttons.

## Power Supply

- Toroidal transformer 1200VA and 17V iron core feeding regulated **50V** and **18V** rails respectively, plus soft-start circuitry (needed after an initial testing inrush blew ten smd fuses at once).
- 50V rail: 5x TIP35C NPN transistors (more than strictly needed by current, added for thermal headroom due to insulators).
- 18V rail: 5x LM7818 in a balanced parallel configuration.
- Each PCB has its own local LM78xx/LT1117 regulation on the motherboard to isolate RF noise between stages — important since the amplifier operates on the same frequencies the MCUs use internally.
- Tested at 800W (80% load) for several hours for thermal verification.

## Specifications

| Parameter | Value |
|---|---|
| Output power | 450–500 W (typ.) |
| Accepted drive power | 7–110 W |
| Bands | 3.5, 7, 10, 14, 18, 21, 27 (CB), 28 MHz |
| Gain | > 17 dB (typ.) |
| DC-RF efficiency | ~50% |
| Modes | FM, SSB |
| Harmonics | ≤ −65 dBc |
| Max. antenna SWR | 4:1 |
| Weight | 21.3 kg |
| Dimensions (DxWxH) | 48 × 44 × 20 cm |

## Known Issues

- Occasional spurious fault triggering under unusual operating conditions
- Secondary MCU can fail to start correctly, due to a Nextion library issue that stalls its main loop
- AM modulation is not supported (breaks the frequency counter)
- Reduced output power on the 28 MHz band, due to a suboptimal low-pass filter design in the current revision
- Higher drive power required on 21 MHz and 28 MHz (12W, compared to 7W on 3.5MHz) due to RF detector attenuation characteristics
- The bad 28 MHz LPF has elevated output SWR, degrading output power/SWR readings on that band
- Automatic antenna tuner's hardware is complete but not yet fully tested as the matching algorithm is a difficult mechanism, still under development 
- Lower than desired efficiency of the amp stage, currently about 50%, which pushes power dissipation to the limit
 
This project is shared as inspiration for design ideas, for other homebrew amp builders. I do not recommend attempting to recreate my exact design as there are many subtle details one must account for when building it that you may not know or find unless you are very thorough and understanding of the design.

