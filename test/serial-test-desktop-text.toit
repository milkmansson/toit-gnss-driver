import uart

/** Echo serial input to serial monitor. */

DEVICE := "COM5"
//DEVICE := "\\\\.\\COM3"
//DEVICE := "COM19" // Windows
//DEVICE := "/dev/ttyACM0" // Linux (example)
BAUD   := 9600
//BAUD   := 115200

safe-decode_ data/ByteArray -> string:
  e := catch: return data.to-string
  // Only reached on invalid UTF-8 — walk and replace.
  result := ByteArray data.size
  i := 0
  out := 0
  while i < data.size:
    b := data[i]
    seq-len := 0
    if b & 0x80 == 0:         seq-len = 1
    else if b & 0xE0 == 0xC0: seq-len = 2
    else if b & 0xF0 == 0xE0: seq-len = 3
    else if b & 0xF8 == 0xF0: seq-len = 4

    if seq-len == 0:
      result[out] = '_'
      out++
      i++
      continue

    valid := true
    if i + seq-len > data.size:
      valid = false
    else:
      j := 1
      while j < seq-len:
        if data[i + j] & 0xC0 != 0x80:
          valid = false
          break
        j++

    if valid:
      k := 0
      while k < seq-len:
        result[out] = data[i + k]
        out++
        k++
      i += seq-len
    else:
      result[out] = '_'
      out++
      i++

  return result[0..out].to-string


main:
  port := uart.Port DEVICE --baud-rate=BAUD
  while true:
    print (safe-decode_ port.in.read)
    sleep --ms=1
