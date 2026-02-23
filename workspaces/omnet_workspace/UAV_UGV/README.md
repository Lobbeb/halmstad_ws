# UAV_UGV OMNeT Run and Plot Commands

Use these commands to run the OMNeT simulation (with or without graphics) and plot results from a single simulation run.

## Run OMNeT (with graphics / Qtenv)

From the project directory:

```bash
cd /home/ruben/omnet_workspace/UAV_UGV
```

WiFi (Gazebo bridge):

```bash
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-WiFi -n src:../inet4.5/src
```

5G profile (Gazebo bridge):

```bash
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-5G -n src:../inet4.5/src
```

LoRa (native `flora`, Gazebo bridge):

```bash
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-LoRa -n src:../inet4.5/src:../flora/src
```

## Run OMNeT (without graphics / Cmdenv)

WiFi:

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge-WiFi -n src:../inet4.5/src
```

5G profile:

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge-5G -n src:../inet4.5/src
```

LoRa (native `flora`):

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge-LoRa -n src:../inet4.5/src:../flora/src
```

## Avoid Overwriting Results (recommended)

Use a unique result directory per run:

```bash
RUN_TAG=$(xxx)
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-WiFi \
  -n src:../inet4.5/src \
  --result-dir="results/$RUN_TAG"
```

## Plot Results (single simulation)

Run these after one OMNeT simulation has finished. Replace `<run_dir>` with the folder you used in `--result-dir` (or `results` if you did not set one).

### 1) Path loss / distance / estimated RSSI (from `.tlog`)

```bash
python3 path_loss_analysis.py --results results/<run_dir> --config-prefix Communication-GazeboBridge-WiFi --plots
```

Important arguments:

- `--results <dir>`: folder containing OMNeT result files (`.tlog`, `.vec`, `.sca`)
- `--config-prefix <prefix>`: config prefix used in OMNeT output filenames (examples below)
- `--plots`: generate plot images (SVG)
- `--csv-dir <dir>`: optional override for CSV output directory
- `--plot-dir <dir>`: optional override for plot output directory
- `file`: optional specific `.tlog` file path/name instead of auto-picking by prefix

Example prefixes:

- `Communication-GazeboBridge-WiFi`
- `Communication-GazeboBridge-5G`
- `Communication-GazeboBridge-LoRa`

### 2) Network metrics (delay, SNIR, PER, throughput, data rate)

```bash
python3 network_metrics_analysis.py --results results/<run_dir> --config-prefix Communication-GazeboBridge-WiFi
```

Important arguments:

- `--results <dir>`: folder containing `.vec` / `.sca`
- `--config-prefix <prefix>`: auto-picks latest matching `.vec` / `.sca`
- `--vec <file>`: use a specific `.vec` file
- `--sca <file>`: use a specific `.sca` file
- `--pathloss-csv <file>`: optional path loss CSV (for RSSI estimate plot overlay)
- `--plot-dir <dir>`: optional override for plot output directory
- `--csv-dir <dir>`: optional override for CSV output directory
- `--write-csv`: also write the large combined metrics CSV (disabled by default)

With path loss CSV linked in:

```bash
python3 network_metrics_analysis.py \
  --results results/<run_dir> \
  --config-prefix Communication-GazeboBridge-WiFi \
  --pathloss-csv results/<run_dir>/Communication-GazeboBridge-WiFi-0_path_loss.csv
```

## Notes

- `Qtenv` is the GUI mode; `Cmdenv` is terminal-only.
- LoRa requires the extra NED path `:../flora/src`.
- By default, the plot scripts now write outputs into the same results folder they read from.
