# robot_drive

Drive-only control: **MQTT → host bridge → UART → MCU**.

Command FIFO and functional safety live in **firmware**. The host only translates MQTT and refreshes the motion watchdog over UART.

## Architecture

```
MQTT clients  →  host/mqtt_bridge.py  →  UART  →  firmware (FIFO + FUSA)
     ↑                  ↑ status publish ←────────┘
debug_web (debug only)
```

`debug_web/` is a **separate debug-only** browser pad (arrows + STOP). It talks MQTT only; it is not part of the production control path.

## Functional safety (MCU)

| Event | Motors | FIFO | Estop |
|-------|--------|------|-------|
| `STOP` | off immediately | **flushed** | latched until `CLEAR` |
| Watchdog TTL expired | off | **flushed** | no |
| Travel limit (FW/BK) — *disabled by default* | off | **flushed** | no |
| Finite turn done / `dur` elapsed / `IDLE` | off current | **kept**; next dequeued | no |

- FW/BK (and continuous turns) with **`dur=0`** need `HB` within `ttl` or the watchdog fires and **flushes the queue**.
- Commands with **`dur>0`** run for that time (MCU self-pets watchdog). They **preempt** the current move / FIFO (fresh segment every time).
- The travel cap is off (`TRAVEL_LIMIT_COUNTS = 0`), so `HB` is the only deadman on held driving. Set it non-zero to also bound distance per hold.
- Queue depth: **`CMD_QUEUE_DEPTH = 8`** (used by `dur=0` sequencing).

## UART protocol

Baud **115200**. Max line **96** bytes.

Arguments are **positional** (space-separated). Trailing args may be omitted (defaults apply).

| Command | Form | Effect |
|--------|------|--------|
| `FW` | `FW [speed] [ttl] [dur]` | Drive forward |
| `BK` | `BK [speed] [ttl] [dur]` | Drive backward |
| `LT` | `LT [speed] [ttl] [counts] [dur]` | Turn left |
| `RT` | `RT [speed] [ttl] [counts] [dur]` | Turn right |
| `HB` | `HB [ttl]` | Refresh watchdog — **no reply on success** |
| `IDLE` | `IDLE` | End segment |
| `STOP` | `STOP` | Halt + flush + estop |
| `CLEAR` | `CLEAR` | Clear estop |
| `ST` | `ST` | Full compact status |
| `RESET` | `RESET` | Zero odometry |

Examples:

```text
FW 80 300 1000     # speed=80, ttl=300ms, dur=1000ms
BK 80              # speed=80, ttl=300 (default), dur=0
LT 70 300 0 1000   # spin left 1s (preferred one-shot, like FW)
RT 70 300 0 1000
LT 70 300 800      # finite turn ~800 travel counts (dur defaults 0 → needs time/WD)
STOP
```

Defaults when omitted: FW/BK speed **100**, LT/RT speed **120**, ttl **300**, counts **0**, dur **0**.

For MQTT one-shots prefer `"dur":1000` (same as forward). Example:

```bash
mosquitto_pub -t robot/drive/cmd -m '{"cmd":"turn_left","speed":120,"ttl":300,"dur":1000}'
```

Replies (kept small on purpose):

| Line | When | Example |
|------|------|---------|
| `A …` | Command ACK only | `A e=fw m=F es=0 q=0 hb=0` |
| `S …` | `ST` only | `S e=st m=I … tr=0 v=7500` |
| `D …` | Motion debug (default off; `DBG 1` on) | `D beg …` / `D mid …` / `D end …` |
| `E …` | Error | `E estop` |
| `RDY` | Boot banner | |

Mode codes: `I F B L R`. Stop codes: `- C W T U D K Q`. `hb=1` means host must send `HB`.

### Motion debug (`D` lines)

Off by default (`DEBUG_DEFAULT=0`). Toggle: `DBG 1` / `DBG 0` (or MQTT `{"cmd":"debug","on":1}`).

| Field | Meaning |
|-------|---------|
| `beg` | Segment start after kick+drive: `m` mode, `sp` speed, `dur`, `c` counts, commanded `mL`/`mR` |
| `mid` | Every ~250 ms: PWM `mL`/`mR`, `tr` travelAbs, `sd` straightDiff, `ws` wheelSpeed, `tl` ms left, `v` battery mV |
| `end` | Halt snapshot: `ls` stop reason, last PWM, `tr`/`sd`, `tk` ticks, `ms` active time, `fl` queue flushed, `v` battery mV |
| `stall` | Encoders frozen while driving → PWM `boost` applied in the same direction |

**How to read a failing repeat FW:** compare good vs bad `end` lines — `mL`/`mR` non-zero + `tr≈0` ⇒ drive commanded but wheels not moving; `ms`≪`dur` ⇒ cut early (`ls=W/T/…`); large `|sd|` with fighting PWM ⇒ straight-assist.

Bridge prints `D` lines to stdout and publishes `{prefix}/debug`.

### `counts` (LT / RT)

Encoder turn target: progress `+= |(ΔdL − ΔdR)|` until `counts`.  
`counts=0` → continuous turn (needs `HB` if `dur=0`).

### Argument bounds

| Arg | Range | Default |
|-----|-------|---------|
| speed | 1…200 | FW/BK 100, LT/RT 80 |
| ttl (latch / HB) | 50…1000 ms | 300 |
| ttl (finite turn) | 500…5000 ms | 2000 |
| dur | 0…30000 ms | 0 (no time limit) |
| counts | 0 = continuous; >0 = finite | 0 |
| Travel cap FW/BK (`dur=0` only) | 0 = off | 0 |
| FIFO depth | 8 | — |

Segment end is silent (no UART traffic). Use `ST` if you need status.

ACK lines carry only `m` / `es` / `q` / `hb`. Speed, `tl` (ttl left), `ls` (last stop), `tr` (travel) and `v` (battery) come from the full `S` line, so the bridge polls `ST` twice a second to keep MQTT status complete.

### Straight drive (FW / BK)

Encoder P-assist (`Config.h`: `STRAIGHT_KP_NUM/DEN`, `STRAIGHT_CORR_MAX`, `STRAIGHT_MIN_TRAVEL`):

- `straightDiff = Σ(dL − dR)` since segment start  
- After min travel: `corr = clamp(…)`, PWM `left = base − corr`, `right = base + corr`  
- Flip `±corr` in `applyDrive` if it steers the wrong way.

## MQTT (host bridge)

```bash
cd robot_drive/host
pip install -r requirements.txt
python mqtt_bridge.py --broker 127.0.0.1 --serial /dev/ttyACM0 --prefix robot/drive
```

| Topic | Direction | Payload |
|-------|-----------|---------|
| `{prefix}/cmd` | subscribe | JSON command (below) |
| `{prefix}/stop` | subscribe | any / empty → immediate `STOP` |
| `{prefix}/hb` | subscribe | `{"ttl":300}` optional manual HB |
| `{prefix}/status` | publish | MCU status JSON |
| `{prefix}/debug` | publish | raw `D …` motion debug lines |

Command examples:

```bash
mosquitto_pub -t robot/drive/cmd -m '{"cmd":"clear"}'
mosquitto_pub -t robot/drive/cmd -m '{"cmd":"forward","speed":80,"ttl":300,"dur":1000}'
mosquitto_pub -t robot/drive/cmd -m '{"cmd":"turn_left","speed":120,"dur":1000}'
mosquitto_pub -t robot/drive/stop -m '{}'
mosquitto_sub -t robot/drive/status -v
mosquitto_sub -t robot/drive/debug -v
```

`cmd` values: `forward`, `backward`, `turn_left`, `turn_right`, `heartbeat`, `stop`, `clear`, `idle`, `status`, `reset`, `debug`.

While `mode != idle`, the bridge sends `HB` every ~150 ms so the MCU watchdog stays satisfied. If the bridge dies, the MCU still stops (and flushes the FIFO) within one TTL.

## Debug web pad (separate app)

Browser gamepad for manual testing only. Hold arrows to drive; release sends `idle`; **Stop** publishes MCU `STOP`. While held, the debug app sends MQTT `heartbeat` every ~150 ms (watchdog refresh).

Needs MQTT broker + `host/mqtt_bridge.py` running.

```bash
cd robot_drive/debug_web
pip install -r requirements.txt
python app.py --broker 127.0.0.1 --port 8090
# binds 0.0.0.0 — open either:
#   http://127.0.0.1:8090
#   http://<board-lan-ip>:8090
```


Controls: on-screen ▲◀▶▼, keyboard arrows / WASD, Space or Escape = Stop, **Clear estop** after Stop.

## Note

Direct wheel drive — stock Balboa tips unless constrained. Same UART/MQTT contract targets a future STM32 stack.

Forward is the loaded direction on this chassis: at `speed` ≈ 80 (about 27 % duty) FW can stall while BK at the same speed drives fine. Anti-stall raises PWM automatically (`Config.h`: `STALL_*`); prefer `speed` ≥ 120 for FW.
