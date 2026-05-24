# OMNeT LoRa Radio Distance

This note documents the current OMNeT++/FLORA radio-distance behavior used by
the ROS2/Gazebo bridge.

## Purpose

The OMNeT metrics path produces two different distance quantities:

- `link_distance`: geometric UAV-UGV distance computed from mapped Gazebo poses
  inside OMNeT. This stays in OMNeT output for analysis/validation only and is
  not republished as a ROS topic.
- `/omnet/radio_distance`: radio-derived distance estimated from RSSI and the
  configured path-loss model. This is the range topic the estimator may consume.

The tracking pipeline must not read UGV pose/odom topics as tracking input.
Using `/omnet/radio_distance` is allowed because it is derived from network
metrics, not from the UGV pose topic.

## Current LoRa Model

Native LoRa runs use FLORA's `LoRaLogNormalShadowing` path-loss model.
`OmnetMetricsServer` reads RSSI from packet PHY tags on successful receptions
and inverts the same mean path-loss equation:

```text
PL = P_tx - RSSI
PL = PL_d0 + 10 * gamma * log10(d_radio / d0) + X_sigma
d_radio = d0 * 10^((P_tx - RSSI - PL_d0) / (10 * gamma))
```

The published value is clamped to at least `0.1 m`.

Current LoRa constants in `omnet_workspace/UAV_UGV/omnetpp.ini`:

- `P_tx = 27 dBm`
- `f_c = 869.525 MHz`
- `d0 = 40 m`
- `gamma = 2.08`
- `PL_d0 = 127.41 dB`
- `sigma = 3.57 dB`

Because the FLORA model includes log-normal shadowing, `/omnet/radio_distance`
is noisy and model-implied. It will publish `NaN` until a real received packet
provides RSSI.

## Config Wiring

The LoRa config uses:

```text
*.metricsServer.publishRadioDistanceEstimate = true
*.metricsServer.radioDistanceModel = "loraLogNormal"
*.metricsServer.loraPathLossD0 = 40m
*.metricsServer.loraPathLossGamma = 2.08
*.metricsServer.loraPathLossPLd0Db = 127.41
*.radioMedium.pathLossType = "LoRaLogNormalShadowing"
*.radioMedium.pathLoss.d0 = 40m
*.radioMedium.pathLoss.gamma = 2.08
*.radioMedium.pathLoss.sigma = 3.57
```

WiFi/5G approximation configs still use `radioDistanceModel = "fspl"`.

## Test Expectations

During initial tests, expect:

- OMNeT-side `link_distance` should become finite as soon as pose bridging works.
- `/omnet/radio_distance` may stay `NaN` until the first successful LoRa packet.
- `/omnet/rssi_dbm` and `/omnet/snir_db` should also stay `NaN` until packet PHY
  tags are observed.
- The bridge line has eight fields:
  `<simtime_s> <rssi_dbm> <snir_db> <per> <radio_distance_m> <pdr> <latency_s> <jitter_s>`.
