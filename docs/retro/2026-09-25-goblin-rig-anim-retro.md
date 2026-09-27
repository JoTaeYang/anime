# Retrospective: Goblin rig → attack animation (2026-09-25)

## Scope
- Rigging (G1 to G10) of the Meshy "Clay Goblin Warrior" and one attack clip (`attack_swing`, A1 to A3), taken all the way into Unity.
- Deliverables are under `work/goblin_swing/rig/` and `unity/AvatarCheck/Assets/Goblin/`.
  - Spec: `work/goblin_swing/d-23-goblin-rig-design.md`, `d-25-goblin-swing-anim-design.md`
  - Plan and execution log: `work/goblin_swing/d-23-goblin-rig-plan.md`
- Method:
  - Headless Blender scripts.
  - Acceptance Gates checked by independent checkers.
  - main (the orchestrator) verified every result; the user gave visual approval.
  - Production and checker work were split between separate subagents.

## Results
- Retopo mesh 8,122 tris, 24 DEF bones, CTRL/MCH control rig (IK/FK, foot roll, space switching, Ready pose).
- Unity Generic export. Blender and Unity agree within 0.0007 mm / 0.07°.
- The whole G1–G10 chain reruns in about 4.5 minutes (`rig/scripts/run_all_gates.ps1`).

## What went well (keep doing)
| Item | Evidence |
|---|---|
| Every step scripted and reproducible | Replacing the hands with ball fists needed only a full rerun (4.5 min) to revalidate G1–G10 |
| Production and checker split, main re-verifies | Caught real defects: connected-bone translation loss, club roll off by 89°, armpit intersection, and others |
| Reference reworked for our rig rather than copied frame by frame | Rig breakage avoided; the animation needed only one correction |
| Key poses first, then [U] approval | Timing and feel agreed before in-betweening |
| User visual review | Found shoulder shrug, fingers, and an over-bent impact pose that the numeric checks missed |

## Why it was slow (causes and numbers)
### 1. Numeric criteria were set before measuring the target (largest cause)
Most rework came from here. Over 20 small correction tasks went back and forth.

| Criterion | Revisions | Why |
|---|---|---|
| G2.16 grip | 2 | radius overestimated / measurement window inside the gap |
| G2.11 seam band | 3 (±20 → ±30 → ±40 mm) | set without measuring the extent of the effect |
| G5.5 intersection | 1 | "unique face count" instead of the intended "minimum face cover" |
| G6.5 pop | 1 | fixed-interval threshold misjudged continuous IK near a straight joint |
| G6.7 foot contact | 4 | z only → reach + z → pivot height + slip → cumulative contact slip |
| G9.6 / G9.8 | 2 each | small holes and near-closed gaps; interpolation-difference threshold |
| G2.11 hand region / grip radius | 2 each | old hand size ignored / no lathe rings in the gap |

The "measure first, then set the threshold" rule was introduced partway through and was not kept to the end.

### 2. The source model was not ready for rigging
- The club was one piece with the body, so it had to be separated and rebuilt as a lathe.
- Mitten hands with fingers did not match the reference. Changing them to ball fists meant a full rerun.
- Straight T-pose limbs made IK extremely sensitive: knee about 35°/mm.
- Legs were 150 mm, which limited foot roll range and crouching.
- The arms plug straight into the body with no shoulder fillet, which made shoulder topology hard.

### 3. Export and Unity defects were found late (G7–G9)
- Export bones had use_connect set, so translation keys were ignored.
- Blender's FBX importer auto-connects bones, which broke the reimport comparison.
- The club's local axes were rolled 89° away from the socket.
- The Unity prefab bounds were computed from one clip only.
- In Unity batch mode, SkinnedMeshRenderer skinning was not applied to renders.
- The Unity license had expired.

These could all have been caught cheaply on day one by one early round trip to Unity with a crude rig.

### 4. Unity run waiting time
- `unity_goblin.ps1` uses `Start-Process -Wait`, which also waits for Unity's leftover dotnet child process. That added about 10 minutes per run, roughly 8 runs, over an hour in total.

### 5. Checker code was copied
- Checkers copied intersection and clearance metric code from each other, so the same bug recurred in several places.
  - Example: there are no lathe vertex rings inside the grip gap, and this caused problems in both G2.16 and G2.11.
- Small criterion fixes sent to agents one at a time made the round trips longer.

## Improvements (by impact)
1. **Give a rig-ready model spec at modeling time.** A-pose, elbows and knees bent 15–20°, props separated, hand style matching the reference, measured leg/arm proportions. See `docs/rig-pipeline-playbook.md` §1.
2. **Run every new numeric criterion as report-only on its first run.** Look at the distribution, then fix the threshold.
3. **Round-trip a crude rig through FBX to Unity on the first day** (walking skeleton).
4. **Put metrics in a shared library:** clearance, visible intersection, contact, branch flip, silhouette comparison with closing. Checkers only call them.
5. **Fix the Unity wrapper:** wait only for the Unity process, batch several checks into one launch, check the license beforehand.
6. **Split gates into blocking and report-only.** For a cartoon game character, silhouette pixel tuning and between-frame interpolation are report-only.
7. **Make the animation tools clip-agnostic:** pose JSON plus a generic in-betweener, with reference side-by-side video generation built in.

## Items carried into the animation phase
- Start work from Ready pose; IK is sensitive near a straight limb.
- If an IK target goes beyond reach and comes back, the elbow or knee bends sharply within one frame (rigtest f371).
- LBS limit: shoulder 130–170°, elbow 140°, knee 130° and wrist bend 60° lose volume. For raising arms overhead, move the shoulder bone as well.
- For every new clip, check Unity bounds and the root-motion policy. The prefab bounds are the union of all clips + 0.1 m.
- The weapon's `hand_l` space is meaningful only when the left arm is FK (avoids a dependency cycle).
