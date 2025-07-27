# UAV Low-Level Control with Custom ArduPilot Integration

## Overview

This project provides a way to create an executable that interfaces with a modified version of ArduPilot to enable low-level control of a UAV (Unmanned Aerial Vehicle).

## How It Works

The modified version of ArduPilot manages communication between the various onboard systems, including:

* Sensors
* Radio control
* Motors
* Battery

ArduPilot sends data to this executable, which allows the implementation of different controllers — in this case, a cascaded PID controller — at a low level.

The output of the controller is sent back to ArduPilot in PWM format to actuate the motors accordingly.

## Features

* Custom executable for UAV low-level control
* Integration with a modified ArduPilot firmware
* Support for cascaded PID control
* Direct motor actuation via PWM feedback
