import uart
import gpio

/** Echo serial input to serial monitor. */

TX-PIN := gpio.Pin 6
RX-PIN := gpio.Pin 7
BAUD   := 9600

main:
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=BAUD
  while true:
    print port.in.read.to-string
