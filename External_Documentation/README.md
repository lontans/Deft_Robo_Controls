# External documentation (local only)

Vendor PDFs, EDS files, and SDK trees live **on disk here** for bench reference. They are **not** tracked in git (see root `.gitignore`).

**In-repo summary (Controls PCB–relevant only):** [`docs/vendor.md`](../docs/vendor.md).

## Expected local layout

| Path | Notes |
|------|--------|
| `Damiao/*.pdf` | DM-J4310 / J4340 MIT docs |
| `RobStride/RS0*/` | RS01–RS04 firmware PDFs |
| `CubeMars/*.pdf` | AK driver docs — pack samples untrusted |
| `ZeroErr/*.eds` | Prefer EDS over form-gated manuals |
| `MCP2518FD/`, `MCP2562_Documentation.pdf` | SPI-CAN + transceiver |
| `Dynamixel/XL330-*.pdf` | Neck datasheets (not the full SDK tree) |
| `SK9822/` | LED hardware/firmware PDFs |
| `STM32/` | Generic FreeRTOS tutorials — optional |

If a PDF is missing locally, restore from the archive branch `archive/pre-plant-platform` or the vendor site; then refresh `docs/vendor.md` if new PCB-relevant facts appear.
