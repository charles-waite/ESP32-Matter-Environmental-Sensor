Session Log

Purpose
- Track notable actions taken by Codex during interactive sessions so contributors can review changes and rationale.

Location
- codex/SessionLog.m

Entries
- Created AGENTS.md contributor guide covering structure, commands, style, testing, and PR guidance.
- Enabled Thread router eligibility (RX-on-when-idle in config; set link mode and router-eligible flag after Matter init).
- Added serial log for Thread role and router-eligible state during sensor output.
- Updated README to reflect ESP-IDF build workflow, Thread routing eligibility, enclosure paths, and GPIO/D-pin mapping.
- Verified builds with idf.py build (completed with warnings in dependencies).
- Documented session context logging requirement in AGENTS.md.
- Added OLED detection to suppress I2C errors when display is absent, with a one-time serial notice.
- Added numeric device software/hardware version defaults derived from chip_project_config.h.
- Added a default Matter NodeLabel to set a friendly device name on first commission.
- Synced CHIP project config defaults into sdkconfig; removed sdkconfig.defaults.lit (not used in this repo).
- Switched BSEC subscription to static IAQ and mapped s-IAQ to Matter AirQuality enum via a dedicated setter (CO2 no longer drives AirQuality).
- Reworked OLED UI layout: large temp/RH line, sIAQ label+value, CO2 only, and pressure with trend arrow; added sIAQ label mapping and removed boot hint.
- Updated sIAQ label strings to match Matter AirQuality enum wording (Very Poor / Extremely Poor).
- Added Kirby bitmap header and OLED screen swap to alternate between sensor view and Kirby image every 30 seconds.
- TODO: Add animated Kirby screensaver (walk on, stop/face front/wave, walk off).
- Added Matter Time Synchronization cluster on root endpoint and OLED brightness auto-dimming based on computed sunrise/sunset for Seattle (zip 98103) using Thread time sync.
- Added serial time debug lines (local PST label, UTC, and time until next sunrise/sunset).
- Added one-time serial log when Time Sync becomes available.
- Enabled CONFIG_ENABLE_SNTP_TIME_SYNC in sdkconfig.defaults so Matter can read real-time clock without Wi-Fi.
- Refactored Matter Time Sync usage to the Matter-Time-Sync-lib submodule (components/matter_time_sync).
- Updated BSEC state save interval to 1 hour (NVS calibration/state persistence).
- Added lazy Time Sync cluster init in Matter-Time-Sync-lib to handle node availability timing.
- TODO: Enable BSEC NVS state save (create NVS namespace and restore calibration state).
