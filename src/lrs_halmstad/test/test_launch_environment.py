from pathlib import Path
import os
import subprocess

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
CLEARPATH_DIR = REPO_ROOT / "src" / "lrs_halmstad" / "clearpath"


def test_clearpath_extra_urdf_path_is_workspace_portable():
    with (CLEARPATH_DIR / "robot.yaml").open(encoding="utf-8") as stream:
        robot_config = yaml.safe_load(stream)

    extra_path = robot_config["platform"]["extras"]["urdf"]["path"]
    assert extra_path == "lidar3d_0_override.urdf.xacro"
    assert not Path(extra_path).is_absolute()
    assert (CLEARPATH_DIR / extra_path).is_file()

    generated_xacro = (CLEARPATH_DIR / "robot.urdf.xacro").read_text(encoding="utf-8")
    assert 'filename="lidar3d_0_override.urdf.xacro"' in generated_xacro
    assert "/home/" not in generated_xacro


def _dry_run(script_command):
    environment = os.environ.copy()
    environment.update(
        {
            "VIRTUAL_ENV": "/tmp/fake-omnet-venv",
            "PYTHONPATH": "/tmp/fake-omnet-python",
        }
    )
    return subprocess.run(
        ["bash", str(REPO_ROOT / "run.sh"), *script_command],
        cwd=REPO_ROOT,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def test_tmux_ros_commands_remove_python_virtual_environment():
    output = _dry_run(
        [
            "tmux_1to1",
            "baylands",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
        ]
    )

    assert "unset VIRTUAL_ENV PYTHONHOME PYTHONPATH" in output
    assert "export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH" in output
    assert "/bin/bash --noprofile --norc -c" in output
    assert "bash -lc" not in output


def test_support_tmux_ros_commands_use_the_same_clean_environment():
    output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
            "hazard_synthetic_enable:=true",
        ]
    )

    assert "unset VIRTUAL_ENV PYTHONHOME PYTHONPATH" in output
    assert "export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH" in output
    assert "/bin/bash --noprofile --norc -c" in output
    assert "bash -lc" not in output


def test_normal_support_startup_reaches_gazebo_without_broad_cleanup():
    output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
        ]
    )
    startup_sources = "\n".join(
        (REPO_ROOT / "scripts" / path).read_text(encoding="utf-8")
        for path in (
            "run_tmux_support_chain.sh",
            "run_tmux_1to1.sh",
            "run_gazebo_sim.sh",
        )
    )

    assert "./run.sh gazebo_sim baylands false" in output
    assert "prelaunch_safety_cleanup" not in startup_sources
    assert "prelaunch_support_cleanup" not in startup_sources
    assert "signal_processes_by_pattern" not in startup_sources
    assert "pgrep -f" not in startup_sources
    assert "prepare_task_state" in startup_sources


def test_support_observation_defaults_to_baylands_weights():
    script = (REPO_ROOT / "scripts" / "run_support_observation.sh").read_text(
        encoding="utf-8"
    )

    assert 'DETECTOR_BACKEND="ultralytics"' in script
    assert 'baylands-leader-v4-3.pt' in script
    assert 'warehouse-v1' not in script
    assert 'if [ -n "$DETECTOR_ONNX_MODEL" ]; then' in script
    assert 'LAUNCH_ARGS+=("detector_onnx_model:=$DETECTOR_ONNX_MODEL")' in script


def test_gimbal_commands_use_active_gazebo_transport_type():
    launch_source = (REPO_ROOT / "src" / "lrs_halmstad" / "launch" / "spawn_robot.launch.py").read_text(
        encoding="utf-8"
    )

    assert launch_source.count("@std_msgs/msg/Float64]gz.msgs.Double") == 2
    assert "@std_msgs/msg/Float64@ignition.msgs.Double" not in launch_source


def test_task5_tmux_disables_dji2_without_changing_legacy_default():
    task5_output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "mode:=follow",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
            "hazard_projector_enable:=true",
        ]
    )
    legacy_output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "mode:=follow",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
        ]
    )

    assert "wait_for_topic_message /dji1/pose" in task5_output
    assert "dji2_enable:=false" in task5_output
    assert "support_bridge_gimbal:=true" in task5_output
    assert "waiting for message on /dji2/pose" not in task5_output
    assert "wait_for_topic_message /dji2/pose" in legacy_output
    assert "dji2_enable:=true" in legacy_output
    assert "support_bridge_gimbal:=true" not in legacy_output


def test_live_typed_flow_profile_fixes_validated_camera_and_safe_defaults():
    output = _dry_run(
        [
            "support_chain_live_typed_flow",
            "dry_run:=true",
        ]
    )

    assert "Mode: follow" in output
    assert "hazard_projector_enable:=true" in output
    assert "support_camera_scan_enable:=true" in output
    assert "support_camera_scan_uavs:=dji1" in output
    assert "support_camera_scan_yaw_center_deg:=0.0" in output
    assert "support_camera_scan_yaw_amplitude_deg:=0.0" in output
    assert "support_camera_scan_pitch_deg:=-60.0" in output
    assert "support_camera_scan_pitch_amplitude_deg:=0.0" in output
    assert "dji2_enable:=false" in output
    assert "aerial_support_layer_enable:=true" not in output
    assert "GUI: false" in output
    assert "warehouse" not in output.lower()


def test_live_typed_flow_profile_requires_explicit_optional_components():
    output = _dry_run(
        [
            "support_chain_live_typed_flow",
            "dji2_enable:=true",
            "aerial_support_layer_enable:=true",
            "dry_run:=true",
        ]
    )

    assert "dji2_enable:=true" in output
    assert "aerial_support_layer_enable:=true" in output


def test_live_typed_flow_profile_rejects_world_and_camera_overrides():
    for forbidden_argument in ("warehouse", "support_camera_scan_pitch_deg:=-20.0"):
        result = subprocess.run(
            [
                "bash",
                str(REPO_ROOT / "run.sh"),
                "support_chain_live_typed_flow",
                forbidden_argument,
                "dry_run:=true",
            ],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )

        assert result.returncode == 2


def test_task6_typed_dji2_fusion_is_separately_opt_in():
    default_output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "mode:=follow",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
            "hazard_chain_enable:=true",
        ]
    )
    output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "mode:=follow",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
            "hazard_chain_enable:=true",
            "dji2_enable:=true",
            "hazard_fusion_dji2_enable:=true",
        ]
    )

    assert "dji2_enable:=false" in default_output
    assert "wait_for_topic_message /dji2/pose" not in default_output
    assert "dji2_enable:=true" in output
    assert "hazard_fusion_dji2_enable:=true" in output


def test_task7_configured_source_quality_routes_without_enabling_dji2():
    output = _dry_run(
        [
            "tmux_support_chain",
            "baylands",
            "mode:=follow",
            "dry_run:=true",
            "tmux_attach:=false",
            "gui:=false",
            "record:=false",
            "hazard_chain_enable:=true",
            "hazard_fusion_quality_weight_confidence:=0.25",
            "hazard_fusion_quality_weight_freshness:=0.25",
            "hazard_fusion_quality_weight_uncertainty:=0.20",
            "hazard_fusion_quality_weight_view:=0.15",
            "hazard_fusion_quality_weight_communication:=0.15",
            "hazard_fusion_dji1_communication_quality:=0.9",
            "hazard_fusion_dji1_communication_penalty:=0.1",
        ]
    )

    assert "hazard_fusion_quality_weight_communication:=0.15" in output
    assert "hazard_fusion_dji1_communication_quality:=0.9" in output
    assert "hazard_fusion_dji1_communication_penalty:=0.1" in output
    assert "dji2_enable:=false" in output
    assert "hazard_fusion_dji2_enable:=true" not in output


def test_task8_quality_fixture_is_bounded_baylands_and_explicitly_two_source():
    output = _dry_run(
        [
            "validate_support_hazards",
            "scenario:=task7_quality",
            "record:=false",
            "dry_run:=true",
        ]
    )

    assert "World: baylands" in output
    assert "dji2 opt-in: true" in output
    assert "Aerial costmap: false" in output
    assert "quality_weight_communication:=0.6" in output
    assert "dji1_communication_quality:=1.0" in output
    assert "dji2_communication_quality:=0.0" in output
    assert "warehouse" not in output.lower()


def test_task8_expiry_fixture_keeps_dji2_and_costmap_disabled():
    output = _dry_run(
        [
            "validate_support_hazards",
            "scenario:=task7_expiry",
            "record:=false",
            "dry_run:=true",
        ]
    )

    assert "dji2 opt-in: false" in output
    assert "Aerial costmap: false" in output
    assert "dji2:" not in output
    assert "--require-expiry" in output


def test_support_hazard_record_profile_remains_image_free_and_timestamped():
    output = _dry_run(
        [
            "record_experiment",
            "baylands",
            "profile:=support_hazard",
            "tag:=task8_dry_run",
            "dry_run:=true",
        ]
    )

    assert "/coord/support/dji1/aerial_hazards" in output
    assert "/coord/support/dji2/aerial_hazards" in output
    assert "/coord/dji0/aerial_hazards" in output
    assert "/coord/ugv/aerial_hazards" in output
    assert "/a201_0000/global_costmap/costmap_raw" in output
    assert "/a201_0000/plan" in output
    assert "/image_raw" not in output
    assert "/depth_image" not in output


def test_shared_operator_routes_sensor_and_detector_options():
    output = _dry_run([
        "tmux_support_chain", "baylands", "mode:=yolo", "dry_run:=true",
        "record:=true", "record_profile:=support_hazard", "omnet:=true",
        "bridge_depth:=false", "bridge_gimbal:=true", "camera_update_rate:=8",
        "detector_latest_frame_only:=false", "radio_range_topic:=/omnet/radio_distance",
        "lora_sf:=8", "lora_bw:=250kHz",
    ])
    lines = output.splitlines()
    spawn = next(line for line in lines if line.startswith("[spawn]"))
    follow = next(line for line in lines if line.startswith("[follow]"))
    assert "bridge_depth:=false" in spawn and "bridge_gimbal:=true" in spawn
    assert "camera_update_rate:=8" in spawn
    assert "bridge_depth:=" not in follow
    assert "detector_latest_frame_only:=false" in follow
    assert "radio_range_topic:=/omnet/radio_distance" in follow
    assert "profile:=support_hazard" in output
    assert "lora_sf:=8" in output and "lora_bw:=250kHz" in output


def test_explicit_route_selects_its_first_spawn_unless_waypoint_overridden():
    output = _dry_run([
        "tmux_1to1", "baylands", "nav2_goals:=parkinglot_east", "dry_run:=true",
    ])
    assert "waypoint:=parkinglot_east_0" in output
    override = _dry_run([
        "tmux_1to1", "baylands", "nav2_goals:=parkinglot_east",
        "waypoint:=parkinglot_west_0", "dry_run:=true",
    ])
    assert "waypoint:=parkinglot_west_0" in override


def test_weights_preflight_reuses_portable_model_resolution_without_ros(tmp_path):
    models = tmp_path / "models"
    model = models / "obb" / "mymodels" / "fixture.pt"
    model.parent.mkdir(parents=True)
    model.touch()
    environment = os.environ.copy()
    environment["LRS_HALMSTAD_MODELS_ROOT"] = str(models)
    for value in ("fixture.pt", "obb/mymodels/fixture.pt", str(model)):
        result = subprocess.run(
            ["bash", str(REPO_ROOT / "scripts/run_1to1_yolo.sh"), "baylands",
             "check_weights:=true", "weights:=" + value],
            env=environment, cwd=REPO_ROOT, text=True, capture_output=True, check=True,
        )
        assert str(model) in result.stdout
        assert "synced generated waypoint" not in result.stdout


def test_network_recording_is_additive_to_support_evidence():
    output = _dry_run([
        "record_experiment", "baylands", "profile:=support_hazard",
        "omnet:=true", "dry_run:=true",
    ])
    for topic in ("/coord/ugv/aerial_hazards", "/a201_0000/plan",
                  "/omnet/packet_delivery_ratio", "/omnet/latency_s", "/omnet/jitter_s"):
        assert topic in output
    assert "/depth_image" not in output


def test_spawn_defaults_and_independent_depth_gate_resolve_without_starting_nodes():
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node

    path = REPO_ROOT / "src/lrs_halmstad/launch/spawn_robot.launch.py"
    spec = importlib.util.spec_from_file_location("spawn_robot_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations["name"] == "m100"
    assert "m100" in context.launch_configurations["camera_frame_id"]
    # Last nodes are the RGB, depth and gimbal bridges. Evaluate conditions only.
    camera, depth, gimbal = [a for a in description.entities if isinstance(a, Node)][-3:]
    context.launch_configurations.update(bridge_camera="true", bridge_depth="false")
    assert camera.condition.evaluate(context)
    assert not depth.condition.evaluate(context)
    context.launch_configurations["bridge_depth"] = "true"
    assert depth.condition.evaluate(context)
    context.launch_configurations["bridge_camera"] = "false"
    assert not depth.condition.evaluate(context)


def test_omnet_current_options_keep_legacy_config_mapping(tmp_path):
    project = tmp_path / "UAV_UGV"
    project.mkdir()
    executable = project / "UAV_UGV"
    executable.write_text("#!/bin/sh\nexit 99\n")
    executable.chmod(0o755)
    (project / "omnetpp.ini").touch()
    setenv = tmp_path / "setenv"
    setenv.touch()
    environment = os.environ.copy()
    environment["OMNETPP_SETENV"] = str(setenv)
    for network, config in (
        ("lora", "Communication-GazeboBridge-LoRa"),
        ("lora-simplex", "Communication-GazeboBridge-LoRa-Simplex"),
        ("lora-duplex", "Communication-GazeboBridge-LoRa-Duplex"),
    ):
        result = subprocess.run(
            ["bash", str(REPO_ROOT / "scripts/run_omnet.sh"),
             "project:=" + str(project), "network:=" + network,
             "lora_sf:=8", "lora_bw:=250kHz", "dry_run:=true"],
            cwd=REPO_ROOT, env=environment, text=True, capture_output=True, check=True,
        )
        assert "Config: " + config + "\n" in result.stdout
        assert "initialLoRaSF=8" in result.stdout
        assert "initialLoRaBW=250kHz" in result.stdout
        assert "flora/src" in result.stdout
