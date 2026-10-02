# Three-Phase Inverter with SVPWM using TI TMS320F28379D

## 📌 Project Overview

This project focuses on the design and implementation of a **three-phase voltage source inverter (VSI)** using **Space Vector Pulse Width Modulation (SVPWM)**. The control algorithm is developed for the **TI TMS320F28379D C2000 microcontroller**.

The project includes the development of SVPWM logic, PWM generation, and embedded C implementation for controlling the inverter switching devices.

## 🎯 Objectives

- Develop SVPWM for a three-phase inverter.
- Generate accurate three-phase PWM signals.
- Implement the control algorithm using **Embedded C**.
- Configure the **ePWM modules** of the TMS320F28379D.
- Generate complementary switching signals for the inverter.
- Analyze inverter output voltage and current waveforms.
- Verify the switching signals through simulation and hardware implementation.

## ⚡ System Description

The three-phase inverter consists of six power switches arranged in three inverter legs.

The SVPWM algorithm determines the appropriate switching states based on the reference voltage vector.

### Main Blocks

```text
DC Voltage Source
       │
       ▼
Three-Phase VSI
       │
       ▼
SVPWM Controller
       │
       ▼
TMS320F28379D
       │
       ▼
ePWM Switching Signals
       │
       ▼
Three-Phase Output
