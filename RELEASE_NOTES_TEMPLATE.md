# Release Notes Template

## Flashing

Two options:

### Option A: App-only (if bootloader + partition table already installed)
Flash only:
- `build/WALL-Env_Sensor.bin`

Example:
```
python -m esptool --chip esp32c6 -b 460800 --before default_reset --after hard_reset \
  write_flash 0x30000 WALL-Env_Sensor.bin
```

### Option B: Full flash (first-time flash or after partition changes)
Flash all three:
- `build/bootloader/bootloader.bin`
- `build/partition_table/partition-table.bin`
- `build/WALL-Env_Sensor.bin`

Example:
```
python -m esptool --chip esp32c6 -b 460800 --before default_reset --after hard_reset \
  write_flash 0x0 bootloader.bin 0x8000 partition-table.bin 0x30000 WALL-Env_Sensor.bin
```

### Optional: Single “full flash” image
Create a combined image:
```
python -m esptool --chip esp32c6 merge_bin -o full_flash.bin \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x30000 build/WALL-Env_Sensor.bin
```

Then flash:
```
python -m esptool --chip esp32c6 -b 460800 --before default_reset --after hard_reset \
  write_flash 0x0 full_flash.bin
```

## Notes
- If the partition table changes, do an `erase_flash` before flashing.
- `idf.py monitor` toggles reset by default; use `--no-reset` if needed.

