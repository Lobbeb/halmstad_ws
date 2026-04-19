#!/usr/bin/env bash

lidar_mode_raw_scan_topic() {
  local lidar_mode="${1:-3d}"

  case "$lidar_mode" in
    2d)
      printf '%s\n' "/a201_0000/sensors/lidar2d_0/scan"
      ;;
    3d)
      printf '%s\n' "/a201_0000/sensors/lidar3d_0/scan"
      ;;
    *)
      echo "[lidar_mode] Unsupported lidar mode '$lidar_mode'. Use 2d or 3d." >&2
      return 1
      ;;
  esac
}

lidar_mode_scan_topic() {
  local lidar_mode="${1:-3d}"

  case "$lidar_mode" in
    2d)
      lidar_mode_raw_scan_topic 2d
      ;;
    3d)
      printf '%s\n' "/a201_0000/sensors/lidar3d_0/scan_from_points"
      ;;
    *)
      echo "[lidar_mode] Unsupported lidar mode '$lidar_mode'. Use 2d or 3d." >&2
      return 1
      ;;
  esac
}

lidar_mode_pointcloud_topic() {
  local lidar_mode="${1:-3d}"

  case "$lidar_mode" in
    2d)
      printf '%s\n' ""
      ;;
    3d)
      printf '%s\n' "/a201_0000/sensors/lidar3d_0/points"
      ;;
    *)
      echo "[lidar_mode] Unsupported lidar mode '$lidar_mode'. Use 2d or 3d." >&2
      return 1
      ;;
  esac
}

lidar_mode_is_2d_scan_topic() {
  [ "${1:-}" = "$(lidar_mode_raw_scan_topic 2d)" ]
}

lidar_mode_is_3d_raw_scan_topic() {
  [ "${1:-}" = "$(lidar_mode_raw_scan_topic 3d)" ]
}

lidar_mode_is_3d_converted_scan_topic() {
  [ "${1:-}" = "$(lidar_mode_scan_topic 3d)" ]
}

lidar_mode_is_3d_scan_topic() {
  local topic="${1:-}"
  lidar_mode_is_3d_raw_scan_topic "$topic" || lidar_mode_is_3d_converted_scan_topic "$topic"
}

lidar_mode_is_3d_pointcloud_topic() {
  [ "${1:-}" = "$(lidar_mode_pointcloud_topic 3d)" ]
}

lidar_mode_is_3d_topic() {
  local topic="${1:-}"
  lidar_mode_is_3d_scan_topic "$topic" || lidar_mode_is_3d_pointcloud_topic "$topic"
}

lidar_mode_infer_mode_from_topic() {
  local topic="${1:-}"

  if lidar_mode_is_2d_scan_topic "$topic"; then
    printf '%s\n' "2d"
  elif lidar_mode_is_3d_topic "$topic"; then
    printf '%s\n' "3d"
  else
    printf '%s\n' ""
  fi
}

lidar_mode_parse_args() {
  local default_mode="${1:-3d}"
  shift || true

  LIDAR_MODE="$default_mode"
  LIDAR_SCAN_TOPIC=""
  LIDAR_POINTCLOUD_TOPIC=""
  LIDAR_REMAINING_ARGS=()
  local lidar_mode_explicit="false"

  while [ "$#" -gt 0 ]; do
    case "$1" in
      lidar:=2d|scan_sensor:=2d)
        LIDAR_MODE="2d"
        lidar_mode_explicit="true"
        ;;
      lidar:=3d|scan_sensor:=3d)
        LIDAR_MODE="3d"
        lidar_mode_explicit="true"
        ;;
      scan_topic:=*)
        LIDAR_SCAN_TOPIC="${1#scan_topic:=}"
        ;;
      pointcloud_topic:=*)
        LIDAR_POINTCLOUD_TOPIC="${1#pointcloud_topic:=}"
        ;;
      *)
        LIDAR_REMAINING_ARGS+=("$1")
        ;;
    esac
    shift
  done

  if [ "$lidar_mode_explicit" = "false" ]; then
    if [ -n "$LIDAR_SCAN_TOPIC" ]; then
      inferred_mode="$(lidar_mode_infer_mode_from_topic "$LIDAR_SCAN_TOPIC")"
      if [ -n "$inferred_mode" ]; then
        LIDAR_MODE="$inferred_mode"
      fi
    elif [ -n "$LIDAR_POINTCLOUD_TOPIC" ]; then
      inferred_mode="$(lidar_mode_infer_mode_from_topic "$LIDAR_POINTCLOUD_TOPIC")"
      if [ -n "$inferred_mode" ]; then
        LIDAR_MODE="$inferred_mode"
      fi
    fi
  fi

  if [ -z "$LIDAR_SCAN_TOPIC" ]; then
    LIDAR_SCAN_TOPIC="$(lidar_mode_scan_topic "$LIDAR_MODE")"
  fi

  if [ -z "$LIDAR_POINTCLOUD_TOPIC" ]; then
    LIDAR_POINTCLOUD_TOPIC="$(lidar_mode_pointcloud_topic "$LIDAR_MODE")"
  fi
}
