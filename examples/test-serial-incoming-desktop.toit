// Copyright (C) 2025 Toit contributors.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import uart
import io

import gnss-driver show *
import nmea-message show *
import nmea-message.nmea-ubx-message show *

DEVICE := "COM19"

BAUD   := 9600
//BAUD   := 38400
//BAUD   := 115200

main:
  // Assemble parser.
  nmea-parser := NmeaParser
  print "NmeaMessage count:  $nmea-parser.registry.size"

  // Open serial communication and start driver.
  print "Opening desktop connection with $BAUD bps"
  port := uart.Port DEVICE --baud-rate=BAUD

  driver := Gnss-driver port.in port.out
  driver.add-parser nmea-parser

  // Print driver start message, and let the driver's own tasks run and simply display incoming messages.
  driver.register-default-lambda (:: | msg | print "$msg")
  print "Driver started..."
