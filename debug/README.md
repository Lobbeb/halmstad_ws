# Debug Topic Audit

Use this to find topic load problems while the stack is running.

## Main command

```bash
./run.sh topic_audit_table mode:=follow out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt sample_s:=5 samples:=10
```

Outputs:
- `debug/debug_topics_table.csv`: full table
- `debug/debug_topics_table.md`: readable summary
- `debug/debug_topics_table.md.visited`: topics that have been sampled

## Common runs

Follow stack:

```bash
./run.sh topic_audit_table mode:=follow out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt
```

Normal YOLO:

```bash
./run.sh topic_audit_table mode:=yolo out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt
```

YOLO tracker:

```bash
./run.sh topic_audit_table mode:=yolo_tracker out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt
```

YOLO visual bridge:

```bash
./run.sh topic_audit_table mode:=yolo_visual_bridge out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt
```

Support YOLO:

```bash
./run.sh topic_audit_table mode:=support_yolo out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt
```

## Watch topics while launching

This catches topics that appear after the audit starts:

```bash
./run.sh topic_audit_table mode:=yolo watch_new:=true sample_s:=30 out:=debug/debug_topics_table.md ignore:=debug/debug_topics_ignore.txt -- ./run.sh tmux_1to1 baylands mode:=yolo
```

## Refresh without sampling

Use this after editing the CSV manually:

```bash
./run.sh topic_audit_table out:=debug/debug_topics_table.md refresh_only:=true
```

## Ignore file

Put regex patterns in:

```text
debug/debug_topics_ignore.txt
```

Example:

```text
/transition_event$
/feedback$
/parameter_events$
/rosout$
/tf_static$
```

## Suspects

Current suspect outputs:
- `debug/debug_topics_suspects.csv`
- `debug/results/debug_topics_suspects.md`

These are filtered from `debug_topics_table.csv` for topics above the current load threshold. Use the `status` and `fix` columns to track what has been checked.
