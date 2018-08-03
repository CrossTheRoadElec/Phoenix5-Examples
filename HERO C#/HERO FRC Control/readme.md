# Using HERO with the FRC roboRIO

This example is a pair of projects - one for the CTRE HERO, and one for the roboRIO.

Using these projects together allows the control of CTRE devices from the HERO while
the roboRIO handles the enable signals.

## Why?

Phoenix already supports the CTRE CAN motor controller and sensor devices for roboRIO.  However there may be advantages in having other sub systems commanding these devices.  

For example, a device can calculate the feed-forwards and target positions of a single complex mechanism such as a multi joint arm or swerve drivetrain.  Then, directly update the involved Talon SRXs without involving the roboRIO.

Different platforms have different languages, toolchains, debugging-features, performance metrics, etc.  This example focuses on the HERO control board, but this method could be used with anything that can link Phoenix.  Additionally developers can port the open source Phoenix-netmf repository.

RoboRIO still sends the FRC-specific enable, ensuring safe operation of these motor controllers.

## Setup
Both the roboRIO and HERO must be connected on the same CAN bus.

Talon SRXs and Victor SPXs should have FRC firmware (v3.X).

## roboRIO CAN Joysticks
This project requires any Phoenix version from FRC 2018.

This project is set up to load the Phoenix libraries and send Gamepad data over the CAN bus.
The roboRIO then handles the enable signals over the CAN bus, allowing other control data to originate from the HERO.


## HERO FRC Control
This project requires at least Phoenix 5.4.3.0.

The project creates a gamepad object that uses CAN bus data as the gamepad values source.
Two Talon SRXs are then driven in a traditional Tank Drive configuration.
