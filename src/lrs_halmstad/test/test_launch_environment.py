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
