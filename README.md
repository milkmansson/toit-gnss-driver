# Basic device agnostic GNSS Driver
Base parser-agnostic driver for GNSS devices.

## Design
`gnss-driver` is a parser-agnostic driver for GNSS receivers in Toit. It handles
the wire-level work — reading bytes from a serial, I2C, or SPI transport,
identifying frame boundaries, and dispatching complete frames to user-supplied
parsers — while leaving message decoding entirely to the user.

To that end, several parsers exist designed to work with this library:
 - NMEA Parser
