# Device- and Protocol-agnostic GNSS Driver for Toit.

Base parser-agnostic driver for GNSS devices.

## Design

`gnss-driver` is designed as a parser-agnostic driver for GNSS receivers in
Toit. It handles the wire-level work — reading bytes from a serial, I2C, or SPI
transport, identifying frame boundaries, and dispatching complete frames to
user-supplied parsers — while leaving message decoding entirely to the user.

To that end, several parsers exist designed to work with this library:

- `toit-nmea-message` — a NMEA protocol parser, with several proprietary NMEA
  message sets included for CASIC, uBlox, and others.
- `ubx-message` — the original uBlox message parser created by the Toit team,
  for handling uBlox binary message types.

The driver does not know how to decode any protocol itself.  Instead, the user
tells it which protocols to take care of by registering a *parser* for each one.
The driver recognises where each frame starts and ends on the wire, hands
complete frames to the supplied parser, and makes the decoded result available.
Messages/Protocols that do not have a parser registered for are skipped cleanly,
so their bytes are never mistaken for the start of a frame we do wish to recieve.

This method allows extensibility, whilst also allowing code to be reduced
significantly by manually removing code for message types that the use case
is not interested in.

## How it works

The driver follows a small number of steps:

1. **Create a transport.** Obtain an `io.Reader` and `io.Writer` for your device.
Over UART these come directly from a `uart.Port` (`port.in` / `port.out`). Over
I2C or SPI, use the `Reader` and `Writer` helper classes included in this
package to wrap a `serial.Device`.

2. **Create the driver.** Constructing a `Gnss-driver` starts an internal
   background task (the *receiver task*) that continuously reads frames from the
   wire. This task starts immediately, so register your parsers promptly — until
   at least one parser is registered, the receive loop simply discards bytes.

3. **Register parsers.** For each protocol you want decoded, call `add-parser`
   with an instance of the parser class for handling those messages.  (Parser
   class must contain `.magic` and `.from-reader` functions.)

4. **Consume messages.** Messsages can be consumed by
   a) Reading the most recent message of a type from `latest-message`, or,
   b) register a lambda to be called as each message arrives,
   c) send a poll and wait synchronously for the reply.

## Quick start

The minimal case — open a serial port, start the driver, and register an NMEA
parser:

```toit
import uart
import gnss-driver show *
import nmea-message show *

main:
  port := uart.Port "/dev/ttyUSB0" --baud-rate=9600

  // Constructing the driver starts the receiver task.
  driver := Gnss-driver port.in port.out

  // Register the NMEA parser against the NMEA magic byte ('$' == 0x24).
  nmea-parser := NmeaParser
  driver.add-parser #[0x24] (:: | r | nmea-parser.from-reader r)
```

At this point the driver is already reading and decoding NMEA frames in the
background. UBX and CASIC frames (and AIS) are skipped automatically, so they
will not corrupt NMEA frame detection even though no parser is registered for
them.

## Registering parsers

A parser is registered with a magic byte sequence and a parse lambda:

```toit
driver.add-parser #[0x24] (:: | r | nmea-parser.from-reader r)
```

The lambda receives the underlying `io.Reader`, must consume exactly one complete
frame, and must return the decoded message object. The driver matches the longest
registered magic at the head of the stream, so multi-byte magics (such as UBX's
`#[0xb5, 0x62]`) coexist with single-byte ones.

To register more than one protocol, call `add-parser` once per protocol:

```toit
nmea-parser := NmeaParser
ubx-parser  := UbxParser   // Illustrative; use the parser the protocol provides.

driver := Gnss-driver port.in port.out
driver.add-parser #[0x24]       (:: | r | nmea-parser.from-reader r)
driver.add-parser #[0xb5, 0x62] (:: | r | ubx-parser.from-reader r)
```

### Skipping protocols you do not parse

The driver ships with built-in skip routines for NMEA, AIS, UBX, and CASIC
framing. Any of those magics that you do *not* register a parser for is skipped
cleanly and automatically — no registration required. This keeps a noisy
multi-protocol stream from breaking detection of the one protocol you care about.

For any other protocol, register a magic as skip-only with `add-skip`, optionally
supplying your own skip lambda:

```toit
// Skip frames of a protocol we do not want to decode, but whose framing we know.
driver.add-skip #[0xAA, 0xBB] --skip=(:: | r | my-skip-routine r)
```

If no skip routine is known for a magic and none is supplied, the driver falls
back to advancing one byte at a time and logs a warning once.

Use `remove-handler magic` to remove any parser or skip registration for a magic.

## Consuming messages

There are three ways to get at decoded messages, and they can be combined.

### 1. Last received message of each type

The driver keeps the most recently received message of every type it has seen in
the `latest-message` map. The map is keyed by each message's `full-name` (for
example `"NMEA-GP-RMC"` or `"NMEA-GN-GGA"`), and each value is the last instance
of that type received:

```toit
rmc := driver.latest-message.get "NMEA-GP-RMC"
if rmc:
  print "Latest position fix: $rmc"
```

Reading this map does not consume the message — repeated reads return the same
instance until a newer message of that type arrives. This is the simplest way to
poll for "the current state" without registering any callbacks.

### 2. A lambda per message type

To be notified as messages arrive, register a lambda against a message `id`. Each
time the receiver task decodes a message whose `id` matches, your lambda is called
with that message:

```toit
driver.register-message-lambda "RMC" (:: | msg |
  print "Got an RMC: $msg")
```

Only one lambda may be registered per `id`; registering again for the same `id`
replaces the previous one. Pass `null` as the lambda to deregister.

### 3. A catch-all default lambda

To handle every message that does not have a specific lambda registered, register
a default lambda. This is useful for logging or building a generic router:

```toit
driver.register-default-lambda (:: | msg |
  print "Unhandled: $msg.full-name -> $msg")
```

While a default lambda is registered it replaces the driver's built-in
debug-level logging of unhandled messages. Pass `null` to deregister.

> **Threading note.** Lambdas registered via either method run on the receiver
> task — the same task that reads and decodes frames. A long-running lambda
> blocks all further message processing until it returns. If you need to do
> significant work in response to a message, dispatch it to a separate task from
> inside your lambda and return immediately.

## Sending to the device

Several send paths are provided depending on the protocol:

- `send-byte-array bytes` — writes the exact bytes given, with no framing,
  termination, or encoding. Use this for pre-built binary frames such as UBX
  commands.
- `send-message message` — writes `message.to-string` followed by a trailing
  CRLF. Suitable for line-framed protocols such as NMEA. Fire-and-forget.
- `send-sentence sentence` — writes a literal string verbatim with a trailing
  CRLF, for hand-built NMEA-style sentences.

### Polling and waiting for a reply

`send-poll-message` sends a message and blocks until the device replies, returning
the matching reply message (or `null` on timeout):

```toit
reply := driver.send-poll-message poll
if reply:
  print "Device replied: $reply"
```

The driver writes the poll, then waits for any incoming message whose `id` is in
the poll's `poll-reply-ids` list. At most one poll may be in flight at a time;
concurrent calls block on an internal mutex until the outstanding poll completes.

The reply returned by `send-poll-message` is delivered only to the caller — it is
*not* also dispatched to any lambda registered for that `id`. If you want the
reply handled by a registered lambda, use `send-message` instead.

Multipart messages are not currently supported as poll requests.

## The message contract

The driver is parser-agnostic, but it does require the message objects your parser
returns to expose a small duck-typed interface. The driver does not check this at
registration time; a missing method will throw at dispatch time. The required
methods are:

| Method | Purpose |
| --- | --- |
| `id -> string` | Identifier used for poll matching and per-id lambda dispatch. Should be unique per message kind within a protocol (e.g. `"RMC"`, `"GGA"`). |
| `full-name -> string` | Key used in `latest-message`. Should be unique across every message a device might emit (e.g. `"NMEA-GP-RMC"`). |
| `is-multipart -> bool` | True if the message is part of a multi-frame sequence. Multipart messages are not supported by `send-poll-message`. |
| `poll-reply-ids -> List?` | For a poll message, the IDs the driver should accept as a reply (typically the message ID, plus any ACK/NAK IDs). May be `null` when the message is not a poll. |
| `to-string -> string` | Wire representation, used by `send-message`. |

If your parser produces objects that do not implement these natively, wrap them in
a small adapter class that does.

## Shutting down

Call `close` to stop the receiver task:

```toit
driver.close
```

## Transports other than UART

For I2C or SPI, wrap a `serial.Device` using the `Reader` and `Writer` helper
classes included in this package, and pass them to the constructor in place of
the UART reader/writer:

```toit
reader := Reader device
writer := Writer device
driver := Gnss-driver reader writer
```
