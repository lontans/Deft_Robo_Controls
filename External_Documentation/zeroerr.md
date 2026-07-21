# ZeroErr eRob — CANopen notes (assembled from public sources, NOT the full vendor manual)

**Read this caveat before trusting anything below.** The official manual
(*"eRob CANopen and EtherCAT User Manual V1.9"*, 219 pages) lives behind a
lead-capture form at `https://book.zeroerr.com/view/chd` — it would not
render without filling out a "get a quote" form, and I did not submit one
(not something to script around). Everything in this file was instead
pulled from **public** ZeroErr/ZeroErrControl GitHub repos and their READMEs
via a web-fetch tool that summarizes pages through a small AI model — I did
not get byte-exact raw file contents back, even when I asked for them
verbatim. Treat every value below as "a public example repo's author
believed this to be true," not as a verified datasheet fact, and re-derive
from the actual EDS file (link below) or the real PDF before using any of
this to command real hardware — same posture this repo already takes with
CubeMars's own unverified placeholders (see `../2026-07-10 workstreams/`).

No firmware or host code in this repo currently uses any of this — see
`App/Inc/plant/plugins/zeroerr.h` in the `2026-07-10 workstreams/` folder,
which stubs `PROTO_ZEROERR` as an intentionally-unimplemented reserved slot
pending real protocol confirmation.

## Sources (fetched 2026-07-20)

| What | Where |
|---|---|
| Gated official manual (inaccessible) | https://book.zeroerr.com/view/chd — "eRob CANopen and EtherCAT User Manual V1.9", 219 pages, form-gated |
| Same manual, direct PDF (untried — found via search, not fetched) | https://www.zeroerr.cn/d/file/download/eRob%20CANopen%20and%20EtherCAT用户手册v1.9.pdf |
| Same manual, older version, hosted by a trade show (untried) | https://www.hannovermesse.de/apollo/hannover_messe_2024/obs/Binary/A1338527/eRob%20CANopen%20and%20EtherCAT%20User%20Manual%20V1.7%20compressed.pdf |
| eRob Lab docs portal | https://zeroerrcontrol.github.io/docs/intro |
| CANopen-over-Linux setup page | https://zeroerrcontrol.github.io/docs/eRob%20driver/CANopen-Linux/ |
| Python CANopen example + **EDS file** | https://github.com/ZeroErrControl/eRob_CANopen_Python |
| ROS2 CANopen example (C++) | https://github.com/ZeroErrControl/eRob_CANopen_Linux |
| EtherCAT (SOEM) example, C — not CANopen, noted for contrast | https://github.com/ZeroErrControl/eRob_SOEM_linux |
| EtherCAT (IGH) example, C++ — not CANopen, noted for contrast | https://github.com/ZeroErrControl/eRob_IGH_EtherCAT |

**The single best source if you want ground truth**: `ZeroErr Driver_V1.5.eds`
inside the Python repo —
https://raw.githubusercontent.com/ZeroErrControl/eRob_CANopen_Python/main/ZeroErr%20Driver_V1.5.eds
— an EDS (Electronic Data Sheet) is CANopen's own standard machine-readable
object dictionary format, so this is as close to "the real spec" as
anything public. **Download and read it directly** (not through an AI
summarizer) before writing any real `zeroerr.c` — this file only reports
what a summary pass over it claimed.

## Device identity (per the EDS, as summarized)

- Product name in the EDS: **"ZeroErr eDriver"** (the drive/servo controller
  eRob actuators use — "eRob" is the actuator/joint module, "eDriver" is its
  CANopen-speaking controller electronics; worth confirming this distinction
  matches whatever specific ZeroErr product line is actually on hand)
- Vendor ID: `0x5A65726F`, Product code: `0x26483052`, Revision: `0x00020111`
  (EDS `[DeviceInfo]`-style fields — not cross-checked against anything else)
- Profile: **DSP402 / CiA 402** (drives and motion control) — same device
  profile family as most other single-frame-CAN actuators in this codebase's
  plugin set, but CANopen's actual wire encoding (SDO/PDO, NMT) is a
  different mechanism than RobStride/Damiao/CubeMars's raw single-frame
  approach — see the integration-shape warning in `zeroerr.h`.
- Claims 4 RxPDO + 4 TxPDO channels.

## Object dictionary — indices seen across the EDS summary + example code

| Index | Name | Notes |
|---|---|---|
| `0x1000` | Device type | CiA 402 mandatory object |
| `0x1001` | Error register | CiA 402 mandatory object |
| `0x1018` | Identity object | CiA 402 mandatory object |
| `0x6040` | **Controlword** | write-only; see bit patterns below |
| `0x6041` | **Statusword** | read-only; bit 10 = target reached (seen in example code) |
| `0x6060` | Modes of operation | `0x01` = Profile Position (seen set explicitly in example code); profile also claims velocity/torque/interpolated modes exist |
| `0x6064` | Position actual value | |
| `0x607A` | Target position | |
| `0x607D` | Position limits | |
| `0x6080` | Max motor speed | |
| `0x606B` / `0x606C` | Velocity demand / actual value | |
| `0x6071` | Target torque | |
| `0x6076` | Motor rated torque | |
| `0x6077` | Torque actual value | |
| `0x6081` | Profile velocity | used in Profile Position mode (example code) |
| `0x6083` / `0x6084` | Profile acceleration / deceleration | |
| `0x2240` / `0x2241` | Manufacturer-specific: dual encoder feedback | not a standard CiA 402 index — ZeroErr-specific |
| `0x22A2` | Manufacturer-specific: drive temperature | ZeroErr-specific |
| `0x2380`–`0x2382` | Manufacturer-specific: current/velocity/position loop PID gain sets | ZeroErr-specific |

Everything `0x60xx` above is standard CiA 402 — if that part of the summary
is accurate, an off-the-shelf CANopen/CiA-402 master stack should mostly
"just work" for position/velocity/torque mode without ZeroErr-specific
glue, aside from the `0x22xx`/`0x23xx` manufacturer extensions.

## PDO mapping + COB-IDs (from the Python example's PDO setup code)

- **TxPDO1** (`0x1800`/`0x1A00`): statusword (`0x6041`) + actual position (`0x6064`)
- **RxPDO1** (`0x1400`/`0x1600`): controlword (`0x6040`) + target position (`0x607A`)
- COB-ID formula observed: `TxPDO1 = 0x180 + node_id`, `RxPDO1 = 0x200 + node_id` —
  this is CANopen's **standard predefined connection set**, not a custom
  ZeroErr scheme, which is a good sign for compatibility with generic tooling.
- Only PDO1 of the claimed 4 RxPDO/4 TxPDO channels was seen actually configured
  in example code; PDO2-4 content is unconfirmed.

## Controlword (0x6040) sequences seen in example code

CiA 402's standard state machine, values as used in the Python example:

| Value | Meaning (as used) |
|---|---|
| `0x06` | Shutdown |
| `0x07` | Switch on / ready |
| `0x0F` | Enable operation |
| `0x1F` | Enable operation + bit 4 set (new setpoint / "start motion" in Profile Position mode) |
| `0x80` (or `0x0080`) | Fault reset |

Enable sequence used: `0x06 → 0x07 → 0x0F`, then set target position and
write `0x1F` to trigger the move. Statusword (`0x6041`) bit 10 was checked
as "target reached" (`statusword & 0x0400`). These are the standard CiA 402
state-machine transitions, not ZeroErr-specific — consistent with the
profile claim above.

## CAN bus parameters — conflicting values observed, not resolved

- Python repo (`eRob_CANopen_Python`) README: **1,000,000 bps** (1 Mbps)
- `CANopen-Linux` setup doc example: `sudo ip link set can0 type can bitrate 500000` → **500,000 bps**
- Node ID examples seen: `2`, `3` (arbitrary example values, not a documented
  factory default)

Do not assume either bitrate is *the* default — both appear as example
values in different docs, possibly just reflecting how each author's bench
happened to be configured. Confirm the actual configured/factory bitrate
per-unit before wiring anything up (ZeroErr's own config tool, if one
exists, or an active bus bitrate scan) — this is exactly the kind of
"unverified until hardware is in hand" gap `zeroerr.h`'s TODO list already
names generically.

## What's still unknown (real gaps, not filled in above)

- Encoder resolution: one README mentioned **524288 pulses/revolution**
  (2^19) for at least one product variant — not confirmed against whatever
  specific eRob model this repo eventually integrates.
- Homing mode, fault/error-code table, EtherCAT-specific detail (the gated
  manual covers both CANopen *and* EtherCAT — this file only chased the
  CANopen half), NMT heartbeat/node-guarding configuration, and RxPDO2-4/
  TxPDO2-4 content are all unconfirmed.
- Whether "ZeroErr eDriver" (what the EDS names) and whatever eRob model
  variant is actually on hand use the *same* object dictionary — the eRob
  product line spans several rotary actuator models (70F/80I/90T/110I/170I
  seen in product-page search results) that may not all share one driver.
- The EDS file itself was only read through a summarizing fetch, not
  byte-exact — re-download and open it directly before trusting index
  numbers for real register writes.
