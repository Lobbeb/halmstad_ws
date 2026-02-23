#!/usr/bin/env python3
"""
Advanced UAV-UGV Communication Analysis Script
Calculates path loss from position data and extracts all communication metrics.
"""

import sys
import math
import re
from pathlib import Path
from collections import defaultdict
import argparse

def parse_position(pos_str):
    """Parse position string like '(x, y, z)'."""
    match = re.search(r'\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', pos_str)
    if match:
        return (float(match.group(1)), float(match.group(2)), float(match.group(3)))
    return None

def calculate_distance(pos1, pos2):
    """Calculate 3D Euclidean distance between two positions."""
    if not pos1 or not pos2:
        return None
    return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

def free_space_path_loss(distance_m, frequency_ghz=2.4):
    """
    Calculate Free Space Path Loss in dB.
    PL(dB) = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
    Where: d=distance (m), f=frequency (Hz), c=speed of light
    Simplified: PL(dB) = 20*log10(d) + 20*log10(f) - 147.55 (for f in GHz, d in m)
    """
    if distance_m <= 0:
        return 0
    frequency_hz = frequency_ghz * 1e9
    path_loss = 20 * math.log10(distance_m) + 20 * math.log10(frequency_hz) + 20 * math.log10(4 * math.pi / 3e8)
    return path_loss

def analyze_communication_log(log_file):
    """Parse communication log and extract path loss and RSSI information."""
    print("\n=== Communication Log Analysis with Path Loss Calculation ===")
    print(f"File: {log_file}\n")
    
    receptions = []
    distance_stats = {
        'min': float('inf'),
        'max': 0,
        'total': 0,
        'count': 0
    }
    
    path_loss_stats = {
        'min': float('inf'),
        'max': 0,
        'total': 0,
        'count': 0
    }
    
    uav_last_pos = None
    ugv_last_pos = None
    
    try:
        with open(log_file, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line or not line.startswith('R '):
                    continue
                
                # Parse: R receiver seq msg_type msg_id S start_time start_pos -> E end_time end_pos
                parts = line.split()
                if len(parts) < 12:
                    continue
                
                receiver = parts[1]
                msg_type = parts[3]
                
                # Extract positions
                try:
                    # Find 'S' and positions
                    s_idx = line.find('S ')
                    if s_idx == -1:
                        continue
                    
                    # Extract start position and end position
                    arrow_idx = line.find(' -> E ')
                    if arrow_idx == -1:
                        continue
                    
                    start_part = line[s_idx+2:arrow_idx]
                    end_part = line[arrow_idx+6:]
                    
                    # Get positions
                    start_pos = parse_position(start_part)
                    end_pos = parse_position(end_part)
                    
                    if not start_pos or not end_pos:
                        continue
                    
                    # Use end positions as final positions
                    if 'ugv' in receiver:
                        ugv_last_pos = end_pos
                    elif 'uav' in receiver:
                        uav_last_pos = end_pos
                    
                    # Calculate distance if we have both positions
                    if uav_last_pos and ugv_last_pos:
                        distance = calculate_distance(uav_last_pos, ugv_last_pos)
                        if distance is not None and distance > 0:
                            path_loss = free_space_path_loss(distance, 2.4)
                            
                            receptions.append({
                                'msg_type': msg_type,
                                'receiver': receiver,
                                'distance': distance,
                                'path_loss': path_loss
                            })
                            
                            # Update distance stats
                            distance_stats['min'] = min(distance_stats['min'], distance)
                            distance_stats['max'] = max(distance_stats['max'], distance)
                            distance_stats['total'] += distance
                            distance_stats['count'] += 1
                            
                            # Update path loss stats
                            path_loss_stats['min'] = min(path_loss_stats['min'], path_loss)
                            path_loss_stats['max'] = max(path_loss_stats['max'], path_loss)
                            path_loss_stats['total'] += path_loss
                            path_loss_stats['count'] += 1
                
                except Exception as e:
                    continue
    
    except FileNotFoundError:
        print(f"Communication log file not found: {log_file}")
        return None, None, None
    
    # Print statistics
    print(f"Total receptions analyzed: {len(receptions)}\n")
    
    if distance_stats['count'] > 0:
        print("=== Distance Statistics (between UAV and UGV) ===")
        print(f"  Minimum distance: {distance_stats['min']:.2f} m")
        print(f"  Maximum distance: {distance_stats['max']:.2f} m")
        print(f"  Average distance: {distance_stats['total']/distance_stats['count']:.2f} m")
        print(f"  Samples: {distance_stats['count']}\n")
    
    if path_loss_stats['count'] > 0:
        print("=== Free Space Path Loss Statistics (2.4 GHz) ===")
        print(f"  Minimum path loss: {path_loss_stats['min']:.2f} dB")
        print(f"  Maximum path loss: {path_loss_stats['max']:.2f} dB")
        print(f"  Average path loss: {path_loss_stats['total']/path_loss_stats['count']:.2f} dB")
        print(f"  Samples: {path_loss_stats['count']}\n")
        
        # Calculate estimated RSSI (assuming 20 dBm transmit power for IEEE 802.11)
        tx_power_dbm = 20
        print("=== Estimated RSSI (at receiver) ===")
        print(f"  (assuming transmit power = {tx_power_dbm} dBm)")
        print(f"  Minimum RSSI: {tx_power_dbm - path_loss_stats['max']:.2f} dBm")
        print(f"  Maximum RSSI: {tx_power_dbm - path_loss_stats['min']:.2f} dBm")
        print(f"  Average RSSI: {tx_power_dbm - (path_loss_stats['total']/path_loss_stats['count']):.2f} dBm\n")
    
    # Group receptions by message type
    by_type = defaultdict(list)
    for r in receptions:
        by_type[r['msg_type']].append(r)
    
    print("=== Path Loss by Message Type ===")
    for msg_type, msgs in sorted(by_type.items()):
        avg_pl = sum(m['path_loss'] for m in msgs) / len(msgs)
        print(f"  {msg_type}: avg={avg_pl:.2f} dB (count={len(msgs)})")
    
    return receptions, distance_stats, path_loss_stats

def analyze_scalar_results(sca_file):
    """Extract statistics from .sca file."""
    print("\n=== Packet Statistics ===\n")
    
    stats = {
        'packets_sent': 0,
        'packets_received': 0,
        'end_to_end_delay': [],
    }
    
    try:
        with open(sca_file, 'r') as f:
            for line in f:
                line = line.strip()
                
                # Look for scalar results
                if line.startswith('scalar ') and 'app[0]' in line:
                    if 'uav.app[0]' in line and 'packetSent:count' in line:
                        try:
                            parts = line.split()
                            value = int(parts[-1])
                            stats['packets_sent'] = value
                        except:
                            pass
                    elif 'ugv.app[0]' in line and 'packetReceived:count' in line:
                        try:
                            parts = line.split()
                            value = int(parts[-1])
                            stats['packets_received'] = value
                        except:
                            pass
                    elif 'endToEndDelay' in line:
                        try:
                            parts = line.split()
                            value = float(parts[-1])
                            stats['end_to_end_delay'].append(value)
                        except:
                            pass
    except FileNotFoundError:
        print(f"Scalar file not found: {sca_file}")
        return stats
    
    # Print found statistics
    if stats['packets_sent'] > 0:
        print(f"Packets Sent (UAV): {stats['packets_sent']}")
    
    if stats['packets_received'] >= 0:
        print(f"Packets Received (UGV): {stats['packets_received']}")
        
        # Calculate packet loss
        if stats['packets_sent'] > 0:
            loss_count = stats['packets_sent'] - stats['packets_received']
            loss_rate = (loss_count / stats['packets_sent']) * 100
            print(f"Packet Loss: {loss_rate:.2f}% ({loss_count} lost out of {stats['packets_sent']})")
            print(f"Packet Reception Rate: {100-loss_rate:.2f}%")
    
    if stats['end_to_end_delay']:
        avg_delay = sum(stats['end_to_end_delay']) / len(stats['end_to_end_delay'])
        min_delay = min(stats['end_to_end_delay'])
        max_delay = max(stats['end_to_end_delay'])
        print(f"\nEnd-to-End Delay:")
        print(f"  Average: {avg_delay*1e3:.3f} ms")
        print(f"  Min: {min_delay*1e3:.3f} ms")
        print(f"  Max: {max_delay*1e3:.3f} ms")
    
    return stats

def _find_results_files(result_dir, config_prefix):
    """Find tlog and sca files for a given config prefix."""
    result_dir = Path(result_dir)

    tlog_candidates = sorted(result_dir.glob(f"{config_prefix}-*.tlog"))
    if not tlog_candidates:
        tlog_candidates = sorted(result_dir.glob("*.tlog"))

    sca_candidates = sorted(result_dir.glob(f"{config_prefix}-*.sca"))
    if not sca_candidates:
        sca_candidates = sorted(result_dir.glob("*.sca"))

    tlog_file = tlog_candidates[0] if tlog_candidates else None
    sca_file = sca_candidates[0] if sca_candidates else None

    return tlog_file, sca_file

def main():
    parser = argparse.ArgumentParser(description="Analyze OMNeT++ results (.tlog/.sca)")
    parser.add_argument("results_dir", nargs="?", default="results", help="Results directory (default: results)")
    parser.add_argument("--tlog", dest="tlog_file", help="Specific .tlog file to use")
    parser.add_argument("--sca", dest="sca_file", help="Specific .sca file to use")
    parser.add_argument("--config-prefix", dest="config_prefix", default="Communication", help="Result file prefix (default: Communication)")
    args = parser.parse_args()

    result_dir = Path(args.results_dir)

    if not result_dir.exists():
        print(f"Error: Results directory not found: {result_dir}")
        sys.exit(1)

    tlog_file = Path(args.tlog_file) if args.tlog_file else None
    sca_file = Path(args.sca_file) if args.sca_file else None

    if tlog_file and not tlog_file.exists():
        print(f"Error: .tlog file not found: {tlog_file}")
        sys.exit(1)
    if sca_file and not sca_file.exists():
        print(f"Error: .sca file not found: {sca_file}")
        sys.exit(1)

    if not tlog_file or not sca_file:
        auto_tlog, auto_sca = _find_results_files(result_dir, args.config_prefix)
        tlog_file = tlog_file or auto_tlog
        sca_file = sca_file or auto_sca

    print("=" * 70)
    print("UAV-UGV Communication Analysis with Path Loss Calculation")
    print("=" * 70)
    print(f"Results dir: {result_dir}")
    print(f"Config prefix: {args.config_prefix}\n")

    if tlog_file:
        analyze_communication_log(tlog_file)
    else:
        print("No .tlog file found; path loss/RSSI calculation skipped.")
        print("Hint: enable recordReceptionLog/recordTransmissionLog to generate .tlog files.\n")

    if sca_file:
        analyze_scalar_results(sca_file)
    else:
        print("No .sca file found; packet statistics skipped.\n")

    print("\n" + "=" * 70)
    print("RSSI/Signal Strength Notes:")
    print("=" * 70)
    print("""
Path Loss represents signal attenuation between transmitter and receiver.
RSSI (Received Signal Strength Indicator) = Tx Power - Path Loss

For IEEE 802.11 with 2.4 GHz and typical Tx power of 20 dBm:
  - At 10m:  RSSI ≈ 20 - 60.4 = -40.4 dBm (strong)
  - At 50m:  RSSI ≈ 20 - 80.4 = -60.4 dBm (good)
  - At 100m: RSSI ≈ 20 - 86.4 = -66.4 dBm (fair)

Use the calculated path loss values above with your transmitter power
to determine expected RSSI at various distances.
""")
    print("=" * 70)

if __name__ == "__main__":
    main()
