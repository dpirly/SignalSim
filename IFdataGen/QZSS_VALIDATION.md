# QZSS L1/L5 Validation

This branch adds basic QZSS IF data generation for L1 C/A and L5. The generated
data has been checked with GNSS-SDR in QZSS-only PVT configurations.

## Inputs

- L1 C/A config: `configs/QZSS_L1CA.json`
- L5 config: `configs/QZSS_L5.json`
- GNSS-SDR L1 C/A config: `gnsssdr-configs/QZSS_L1CA.conf`
- GNSS-SDR L5 config: `gnsssdr-configs/QZSS_L5.conf`
- 2025 mixed BRDC RINEX navigation files:
  - `../EphData/BRDC00IGS_R_20250010000_01D_MN.rnx`
  - `../EphData/BRDC00IGS_R_20253650000_01D_MN.rnx`

The static receiver position in the sample configs is:

- Latitude: `35.6586 deg`
- Longitude: `139.7414 deg`
- Height: `50 m`

## Build

From `IFdataGen`:

```bash
cmake --build build -j2
```

## Generate IF Data

From `IFdataGen`:

```bash
./build/IFdataGen -c configs/QZSS_L1CA.json
./build/IFdataGen -c configs/QZSS_L5.json
```

## GNSS-SDR Validation

From `IFdataGen`, using the local GNSS-SDR checkout built at
`../../gnss-sdr/build/src/main/gnss-sdr`:

```bash
../../gnss-sdr/build/src/main/gnss-sdr --config_file=gnsssdr-configs/QZSS_L1CA.conf
../../gnss-sdr/build/src/main/gnss-sdr --config_file=gnsssdr-configs/QZSS_L5.conf
```

Observed QZSS-only PVT checks:

- L1 C/A: 4 QZSS observations, final sample near
  `Lat = 35.658519 deg, Long = 139.741445 deg, Height = 72.90 m`.
- L5: 5 QZSS observations after CNAV clock/iono messages are available, final
  sample near `Lat = 35.658394 deg, Long = 139.741326 deg, Height = 84.92 m`.

L5 uses a 150 second sample file so that GNSS-SDR receives CNAV ephemeris,
clock/iono, and a later ephemeris refresh before evaluating stable PVT. Earlier
solutions can be biased while the receiver has ephemeris but has not yet refreshed
its PVT map with CNAV clock parameters.

## Health Handling

QZSS `SV health` in RINEX is signal-specific. The generator keeps the normal
health filtering for non-QZSS constellations and applies QZSS checks according to
the requested signal:

- L1 C/A rejects records with the L1 C/A health bit set.
- L5 rejects records with the L5 health bit set.

This avoids treating QZSS usage indication bits as a whole-satellite unhealthy
state.
