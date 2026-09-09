# AGENTS.md

## Scope and session start

- `halmstad_ws` is the active development repository.
- EiraX is the downstream UGV integration target for Basuedo. Unless a task explicitly authorizes EiraX changes, active development remains in `halmstad_ws`.
- Cross-repository inspection or changes require explicit task scope.
- Read each target repository's instructions before working there.
- Check branch, HEAD, working tree and relevant running processes before substantial work.
- Do not assume historical environment findings or validation results still apply.
- Read the relevant roadmap, status, handoff and implementation documentation.
- User instructions define intended behavior; inspect code to establish current behavior.
- Report disagreements between code, documentation and instructions before resolving them.

## Architectural boundaries

- Preserve C1–C4 and existing leader/follow/perception behavior unless explicitly changing them.
- Nav2 remains the autonomous UGV mission-motion authority.
- Reuse the existing waypoint driver, localization, planner and controller integration.
- Preserve downstream safety controls; do not bypass them through a new motion path.
- UAV support supplies structured observations/hazards to the existing UGV stack.
- Do not introduce UAV-driven UGV `cmd_vel` control.
- Never use UGV pose, odometry or target ground truth as operational UAV tracking input.
- Operational tracking must use observational sources or permitted network metrics.
- UGV navigation may use its own localization and odometry.
- Keep debug/evaluation ground truth separate from operational tracking inputs.
- Existing C1–C4 baselines, including any use of odometry or simulation truth, must not be silently changed. Identify their purpose and ask before altering them.
- Their existence does not authorize operational leakage or establish compliant tracking.
- Report conflicts rather than silently removing baselines or extending their use.
- Document simulation infrastructure and offline annotation uses of ground truth separately.
- Preserve intended opt-in behavior for typed support hazards and the aerial costmap layer.
- Keep the dji2 typed hazard input disabled by default.
- Enabling that input by default requires validation evidence and explicit approval.
- This restriction does not disable unrelated legacy dji2 functionality.
- Keep Track A navigation validation separate from Track B perception/dataset development.
- Treat Track B as deferred unless explicitly activated in the approved task scope.
- Follow the planning documents for the gates connecting the two tracks.

## Change discipline

- Reuse existing code and helpers before creating parallel systems.
- Prefer small, targeted edits; minimize hardcoding and unnecessary files or ROS topics.
- Preserve topic names, namespaces, parameter names and launch contracts.
- Retain shell `name:=value` conventions and keep operator arguments short.
- Keep YAML defaults and launch-time overrides aligned.
- Reuse follow helpers and the existing Nav2 waypoint-loading path.
- Present a concrete plan before substantial changes and obtain user approval.
- Broad `main` → `support-chain` reconciliation requires a relevant-delta audit,
  dependency/behavior justification and explicit approval.
- Do not copy upstream defaults merely because they are newer.
- Do not commit, push, merge or rebase unless requested.
- Preserve unrelated local changes and collaborators' work, including Ruben's parallel work.
- Ask before removing existing files, code or workflows.
- Do not treat suspected breakage as blanket permission for architectural rewrites.

## Execution, validation and evidence

- The user runs simulations interactively; provide explicit operator instructions.
- Do not start unattended/background simulations.
- Track processes started for the task and clean them up when finished.
- Cleanup applies only to task-owned processes; do not stop pre-existing user/team sessions.
- Avoid broad ROS/Gazebo/tmux kill scripts unless their scope is explicitly authorized.
- After changes, run the smallest meaningful validation for the affected behavior.
- Use Python compilation and shell syntax checks where applicable.
- Build affected ROS packages and relevant dependencies when package changes require it.
- Check changed launch/node contracts with supported help or argument inspection.
- Use focused behavioral tests when syntax/build checks cannot establish correctness.
- Topic publication, RViz display and planner-only tests do not prove runtime navigation.
- Full-runtime claims require evidence of actual motion, goal completion and relevant clearing.
- Distinguish observed results, configured behavior, assumptions and unverified claims.

## Documentation and reporting

- Update the authoritative affected documentation after workflow, contract or logic changes.
- Keep README entrypoints accurate; put detailed explanations in deeper documentation.
- Maintain progress, decisions and validation status in the planning/status files below.
- Report exact changed files with relevant lines or important parameter blocks.
- Explain behavior changes, validation performed and remaining limitations.

## Repository and documentation index

- Operator entrypoints: `./run.sh`, `./stop.sh`; implementations: `scripts/`.
- Main package: `src/lrs_halmstad`; overview: `README.md`, `src/lrs_halmstad/README.md`.
- Runtime: `src/lrs_halmstad/lrs_halmstad/`; launches: `src/lrs_halmstad/launch/`.
- Follow helpers: `src/lrs_halmstad/lrs_halmstad/follow/{follow_core,follow_math}.py`.
- UGV driver: `src/lrs_halmstad/lrs_halmstad/nav/ugv_nav2_driver.py`.
- Typed interfaces: `src/lrs_halmstad_interfaces`; costmap plugin: `src/lrs_halmstad_nav_plugins`.
- Configuration: `src/lrs_halmstad/config`; saved maps and waypoint CSVs: `maps/`.
- Waypoint YAMLs: `src/lrs_halmstad/config/baylands_waypoints/`.
- Planning directory: `William/Replanning_&detection_markdowns/`.
  - `00_MASTER_ROADMAP.md`: workstream scope and gates.
  - `02_UGV_REPLANNING_VALIDATION.md`: Track A methodology and evidence requirements.
  - `01_OBJECT_DETECTION_WORKFLOW.md`: Track B methodology when authorized.
  - `03_STATUS_AND_DECISIONS.md`: progress, decisions and unresolved questions.
  - `05_RESUME_HANDOFF.md`: resumption context; consult closure reports when present.
- Detailed guides/handoffs: repository-relative `descriptions/` when present.
- Resolve documentation paths against the current checkout; report stale or missing links.
