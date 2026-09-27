# Mocap pipeline (kimodo text-to-motion → Unity clip)

Turns a text prompt into a clip that plays on one of our characters.

```
prompt ──► kimodo.cpp ──► kimodo_to_bvh.py ──► retarget.py ──► check_penetration.py
                                                                check_gait.py
                                                                     │
                                                              export_clip.py ──► Unity
```

## Running it

```powershell
.\scripts\run.ps1 mocap -Job cc          # retarget + both gates + FBX export
.\scripts\run.ps1 mocapcheck -Job cc     # gates only, against the saved .blend
```

Jobs live in the `$MocapJobs` table at the top of `scripts/run.ps1`: source BVH,
target model, output `.blend`, output clip FBX, and the shoulder abduction.

## Generating a new motion

`scripts/mocap/kimodo_to_bvh.py` is plain Python (no Blender) and reads the two
raw streams kimodo writes beside each generated animation:

```powershell
python scripts\mocap\kimodo_to_bvh.py <kimodo>\demo-output\<id> assets\mocap\<name>.bvh
```

## The gates, and why they exist

### `check_penetration.py` — arms through the torso

**This is the gate that was missing, and it cost us.** The first CC walk passed
every joint-angle check with the arms intersecting the body on 49 of 90 frames.
Angle verification proves the rotations transferred correctly and says nothing
about whether the result looks right, because the source skeleton has no volume.

A rotation-only retarget cannot know the target has shorter arms or a fuller
torso than the body the motion was generated for. On the CC character the
shoulder spans nearly match (SOMA 0.330m vs CC 0.322m) but the arms are 11%
shorter (0.558m → 0.499m), which drops wrist clearance from 19.7cm to 17.9cm —
enough for the hands to swing into the hips.

**The required clearance is a property of the target, not of the motion, so this
must be re-run for every new character.** `--abduct auto` solves for it.

Three earlier detection attempts measured nothing useful. Recorded so they are
not retried:

| Attempt | Why it failed |
|---|---|
| Sample the bone centreline | A bone is always inside its own limb; misses flesh that overlaps while the bone does not |
| Ray-parity against `Body` | The arms are part of `Body`; degenerate |
| Ray-parity against Shirt/Shorts | Parity needs a closed surface; a shirt is an open tube. Produced 69cm "depths" on a 1.4m character |

What works is `BVHTree.overlap()` face-pair intersection, which assumes nothing
about closedness. Four details make it behave:

- **Gate on depth, not on whether an intersection exists.** A face-pair count
  says a crossing is present and nothing about whether anyone would see it.
  Measured on the same clip:

  | Clip | Frames touching | Max depth | Mean |
  |---|---|---|---|
  | CC, no correction — the defect a human spotted on sight | 39/90 | **25.1mm** | 8.9mm |
  | our character, 8° opening | 14/90 | **3.8mm** | 1.8mm |

  An order of magnitude apart. 25mm on a 1.4m body is a hand inside a hip;
  3.8mm on a 1.9m body is a graze. `MAX_DEPTH` (5mm) sits in that gap.
  Gating on "any intersecting face" failed the second clip, which is wrong.
  Depth is signed against the nearest body face normal — parity would need a
  closed surface, which the skin is not.
- **Runtime-simulated appendages are excluded.** The skirt, scarf and tail are
  spring bones in Unity; the Humanoid clip carries no curves for them, so in
  Blender they sit at rest for the whole clip. On our character they were
  **71% of all "body" faces** (318k of 448k) and dominated the result. Testing
  an arm against cloth posed where it will not be at runtime answers nothing.
  The chain list comes from the active profile's `appendage_bone_rename()`.
  This does not claim an arm through the skirt is fine — only that this gate
  cannot judge it; whether the arm should be a collider for the spring chains
  is a separate runtime question.

- **Arm means forearm downwards.** The deltoid and armpit rest against the ribs
  in every pose, so including the upper arm reports 90/90 intersecting on a clip
  that renders clean. Clavicle-to-elbow is neutral, in neither set.
- **A face counts only if *every* vertex is on one side.** Faces straddling the
  boundary fall out, leaving a gap band that stops the continuous armpit surface
  from registering as a permanent hit.

The opening the solver applies rotates the arm about `arm direction × outward`,
recomputed per frame — **not** about a fixed world axis. The first version used
the fixed travel axis, and when the arm is swung forward or back (exactly the
frames that collide) the limb lies along that axis, so the rotation degenerates
into a roll and moves the hand nowhere. That is why the first sweep plateaued
and then got *worse* at 20°.

### `check_gait.py` — does it walk like a walk

Gates on foot skate, ground penetration, knee range, cadence and hip
oscillation. Symmetry and the loop seam are *reported but not gated*: both are
properties of the generated source clip rather than of the transfer, so gating
them would fail every clip kimodo produces.

The plant threshold is 1cm, not 2.5cm. Mid-swing toe clearance is only a couple
of centimetres, so a loose threshold counts a foot in flight as planted and
reports its travel as skate — that mistake produced a "24cm skate" reading on a
clip that actually skates 0.2cm/frame.

The knee gate (max flexion ≥ 40°) is the "walks like a person with no knees"
check.

## Per-character results

| Job | Rig | Solved abduction | Deepest | Gates |
|---|---|---|---|---|
| `cc` | Character Creator 3/4 | 5° | 3.8mm (limit 5mm) | both pass |
| `character` | our Rigify/Unity export | 7° | 3.6mm (limit 5mm) | both pass |

The CC angle came down 10° → 7° → 5° as the check got less wrong: 10° was
picked by eye, 7° by solving against "no intersection at all", 5° by solving
with the corrected opening axis against a depth limit. Every degree removed is
pose the motion did not ask for, and at 5° the arms hang naturally instead of
being held out.

`character` took three corrections to get an honest answer, and each of the
first two produced a *plausible but wrong* verdict:

1. Abduction about a fixed world axis. Cleared 60/90 at best and got worse at
   20°, which read as "this motion does not fit this character". It was the
   operator that was wrong, not the motion.
2. Cloth counted as rigid body. The skirt and scarf were 71% of all body faces,
   so the result was mostly measuring contact with geometry that is not where
   the runtime puts it. Excluding them moved the baseline from 14/90 to 52/90.
3. Gating on intersection rather than depth. The remaining contacts turned out
   to be 3.8mm at worst — invisible. Gating on depth passes the clip.

Ruled out as an artefact along the way: the same classifier on the same mesh
reports **0 hits in the rest pose**.

## Known limits of the current output

Fixed, but worth knowing:

- **Abduction is a workaround, not a solution.** It adds a constant offset to
  the whole clip, so the arms are held wider than necessary in the frames where
  they were already clear. A collision-aware IK pass is the real fix.
- **Asymmetry ~5° mean / ~19° max** is in the generated source and reads as a
  slight limp.
- **The clip does not loop**; the seam is ~23° across the full range.
- No finger animation and no secondary motion.

## Traps

- The **action's** frame range is authoritative, not the scene's. A fresh BVH
  import leaves the scene at Blender's default 250 frames and the trailing
  frozen poses poison speed, cadence and symmetry.
- Rest-pose alignment needs a **full orthonormal frame**, not minimal-arc
  direction matching. Direction alone leaves roll undefined, which shipped
  forearms rolled over with the palms facing outward — and a direction-only
  verification reported 0.0° error the whole time.
- A CC FBX arrives at 0.01 scale; `retarget.py` bakes that away up front.
- kimodo walks its characters with the chin ~14° up. That is in the generated
  data; `retarget.py` cancels the constant part and keeps the per-frame motion.
