# DeClare

Derivable Existence: identity from WiFi Channel State Information.

An ESP32-C6 captures per-subcarrier I/Q from ambient WiFi, computes vitals on-device and streams them over UDP. `declare.html` turns a live capture into a declaration hash and writes the declared observer back to the device.

No cameras. No wearables. No accounts.

## Contents

```
declare.html          Declaration page, node setup and browser flasher (firmware embedded)
firmware/             ESP-IDF project, target esp32c6
firmware/bin/         Prebuilt images and flash_args
```

Nothing network- or person-specific is compiled in. WiFi, target and node ID are set per device.

## Flash

### From the browser

Open `declare.html` in Chrome or Edge (Web Serial). Enter the WiFi SSID and password, press FLASH and pick the serial port. It writes:

| Offset | Image |
|--------|-------|
| `0x0` | bootloader |
| `0x8000` | partition table |
| `0x9000` | NVS with the WiFi entered on the page |
| `0x10000` | app (`declare.bin`) |
| `0x1F0000` | spiffs |

### From the command line

```bash
cd firmware/bin
esptool.py --chip esp32c6 write_flash @flash_args
```

Then set WiFi with NODE SETUP (below). With no SSID the node skips WiFi and waits for setup.

## Configure

In `declare.html`, connect over USB and use NODE SETUP: SSID, password, target IP, target port and node ID are written to NVS and the node reboots.

The same is available as JSON lines over USB serial at 115200 baud:

```
{"cmd":"get_config"}
{"cmd":"set_config","ssid":"...","password":"...","target_ip":"...","target_port":5005,"node_id":1}
```

Other commands: `csi_read`, `vitals`, `bio`, `brainwave`, `touch_read`, `touch_start`, `touch_stop`, `declare`.

## Declare

1. Enter a name; its letter sum, folded into 1–256, is the name's position.
2. Press Declare and start the capture: 13 ticks of live CSI, with an optional written declaration.
3. The chain runs on the capture: 11 derivative layers, any unmeasured layer seeded from the canonical root, a palindromic fold (SHA3-256 forward XOR SHA3-256 of the reversed bytes), 13 ray perspectives, the field product, anchored by the root's dlog. The result is the hash, genome, position, inverse and ray.
4. The declaration is written to the device over USB (`declare`), stored in NVS, shown on the display and restored at every boot. `get_config` reads it back as `obs_name`, `obs_pos` and `obs_hash`.

When the page is served by a WiBand server it also posts the declaration to `/wiband2/api/v1/identity`.

## Build from source

ESP-IDF v5.3.5:

```bash
cd firmware
idf.py set-target esp32c6
idf.py build
```

Defaults until set on the device: no SSID, target `192.168.1.100:5005`, node 1.

Partition table: `nvs` 0x9000 (24K), `phy_init` 0xF000 (4K), `factory` 0x10000 (1920K), `storage` spiffs 0x1F0000 (64K).

## What it sends

UDP to the target, little-endian, each packet starting with a 4-byte magic and a 1-byte node ID: CSI frames (`0xC5110001`), vitals (`0xC5110002`), features (`0xC5110003`), brainwave bands (`0xC5110009`), per-person vitals (`0xC511000A`), optical (`0xC511000B`), touch (`0xC511000C`).

## The field

GF(257), P = 2⁸ + 1, generator g = 3: 256 nonzero elements, 12 rays (discrete log mod 12).

## License

MIT
