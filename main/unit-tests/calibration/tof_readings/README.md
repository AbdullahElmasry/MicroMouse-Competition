# Three-ToF hardware test

Open `tof_readings.ino`, upload using your existing ESP32 board configuration, and open Serial Monitor at **115200 baud**. Press board RESET to capture startup diagnostics. Requires the same installed VL6180X and VL53L1X libraries as the movement sketches. Motors remain disabled throughout.

| Sensor | Type | XSHUT pin | Assigned address |
|---|---|---|---|
| Left | VL6180X | 4 | 0x30 |
| Right | VL6180X | 5 | 0x31 |
| Front | VL53L1X | 16 | 0x32 |

The sketch uses `Wire.begin()` with board-default pins and clock, matching `main.ino`. On your current board the shared bus uses SDA **21** and SCL **22**. The MPU can remain connected; this sketch does not initialize it.

Initialization follows `main.ino`: all XSHUT pins LOW for 10 ms; wake left, right, then front with 50 ms delays. Each side sensor calls init(), configureDefault(), setAddress(), then setTimeout(200). The front calls init(), setDistanceMode(Medium), setAddress(0x32), setTimeout(200), then startContinuous(30). There is no extra measurement timing budget, forced bus clock, pre-initialization scan, or early return between these steps. Status snapshots are reported after the sequence, followed by a bus scan. Sensors are not shut down based on intermediate status checks. An assigned-address acknowledgement permits side-sensor diagnostic reads; the front additionally requires successful init(). These checks do not prove every configuration register is correct; inspect the reading validity too.

Readings print approximately every 500 ms (longer if sensors time out), with distance in millimeters, validity, range status, timeout flag, and I2C status. Side-sensor range status 0 means no range error. Front-sensor statuses include readable descriptions. Nonzero I2C codes indicate communication problems; a range error without an I2C error can instead mean an unsuitable target or range. Send `s` for another bus scan; use board RESET to retry initialization.

To verify:

1. Confirm each assigned-address probe reports OK, front init reports OK, and the final scan includes 0x30, 0x31, and 0x32.
2. Place a flat matte target about 50 mm from each side sensor in turn, then about 80 mm away. Measure from the sensor face; confirm the correct labeled reading changes and remains VALID.
3. Repeat for the front sensor at about 100, 200, and 400 mm. Record measured versus reported distances rather than assuming exact accuracy.
4. Move the target in front of only one sensor at a time to confirm left/right/front labels match physical placement.
5. If initialization fails, capture the entire startup log, including the final scan and any FAILED stages. If readings fail, include a complete LEFT/RIGHT/FRONT reading group.

This is a manual hardware diagnostic, not an automated unit test. An invalid distance is never a passing measurement. The sketch has not yet been compiled for the board or run on hardware here.
