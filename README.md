# sw20pscontrol

An open-source control module that emulates the Toyota MR2 SW20 Electric Power Steering (EPS) ECU, interfaces with the rest of the in-vehicle electronics.
This project reproduces key logic from the factory system — handling ignition, PSCT (crank cut), speed, steering angle, torque, and diagnostic signals — to enable bench testing, custom EPS control, or retrofit applications.

Refer to this article for the precise explanation of each signal, and its logic
https://www.instructables.com/Mr2-Sw20-Power-Steering-Ecu-Reverse-Engineering/

Features

Interrepts EPS input signals:  ss1, ss2, spd, chk, ifb, iovr and idup, ef1, bms and mth terminals

Drives output signals: wl, mrly, ictr

Monitors Motor Driver Module, MTH (motor voltage) and BMS (brush wear) circuits

Provides diagnostic output (WL) compatible with factory cluster

Supports test bench and standalone EPS operation and communicates via bluetooth

Built for MicroPython compatible microcontrollers esp-32 or pico 2

Use cases

- EPS bench testing and diagnostics
- Custom vehicle retrofits or conversions
- Learning resource for Toyota’s early EHPS logic
