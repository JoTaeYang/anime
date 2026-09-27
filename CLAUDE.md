# Project rules (orca/anime)

## Character rigging and animation work
- For a new character or clip, read `docs/rig-pipeline-playbook.md` first. Before rigging a new character, check the model against the "rig-ready model spec" (§1) and point out anything that fails it.
- Every new numeric Gate criterion is report-only on its first run. Look at the measured distribution before fixing the threshold, and record the basis in the spec.
- Split gates into blocking and report-only. Block only on problems that are visible or break things.
- Do not trust a checker's ok as-is. main runs it itself, compares input sha256, and checks the renders directly. The user judges visual quality.
- Source assets (Downloads, UVReviewProjects) are read-only.
