import uart

/** Echo serial input to serial monitor, using RAW for troubleshooting. */

DEVICE := "COM5"
//DEVICE := "\\\\.\\COM3"
//DEVICE := "COM19" // Windows
//DEVICE := "/dev/ttyACM0" // Linux (example)
BAUD   := 9600
//BAUD   := 115200

safe-decode_ data/ByteArray -> string:
  e := catch: return data.to-string
  // Log raw hex on decode failure to identify mystery messages.
  hex := List data.size
  data.size.repeat: | i | hex[i] = "$(%02x data[i])"
  return "RAW[$data.size]: $(hex.join " ")"

main:
  port := uart.Port DEVICE --baud-rate=BAUD
  while true:
    print (safe-decode_ port.in.read)
    sleep --ms=1
