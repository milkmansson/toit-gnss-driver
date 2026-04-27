# Device- and Protocol-agnostic GNSS Driver for Toit.
Base parser-agnostic driver for GNSS devices.

## Design
`gnss-driver` is a parser-agnostic driver for GNSS receivers in Toit. It handles
the wire-level work — reading bytes from a serial, I2C, or SPI transport,
identifying frame boundaries, and dispatching complete frames to user-supplied
parsers — while leaving message decoding entirely to the user.

To that end, several parsers exist designed to work with this library:
 - toit-nmea-message - a NMEA protocol parser, with several proprietary NMEA message sets included for CASIC, uBlox, and others.
 - ubx-message - the original uBlox message parser created by the Toit team.
