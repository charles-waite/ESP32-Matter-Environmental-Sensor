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
