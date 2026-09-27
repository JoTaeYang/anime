# Goblin Club-Swing — Reference Video Analysis

Source: `KakaoTalk_20260920_192404323.mp4` (copied to `ref/reference.mp4`)
Analysed: 2026-09-20. All measurements are **screen-space**, image 448x576, origin top-left.
Normalized coords are `x/448, y/576` (so 0,0 = top-left, 1,1 = bottom-right).

> **Left / right convention used everywhere below.** The character faces the camera for the
> whole clip. Therefore **screen-left = the character's RIGHT side**, and
> **screen-right = the character's LEFT side**. Both are always stated explicitly.

---

## 1. Video facts (ffprobe + mpdecimate)

| Property | Value |
|---|---|
| Resolution | 448 x 576 (portrait, 7:9) |
| Video codec | h264, Constrained Baseline, yuv420p, progressive |
| `r_frame_rate` | `24/1` |
| `avg_frame_rate` | `24/1` |
| `nb_frames` | **39** |
| Duration | 1.625 s (video stream); container 1.632 s |
| Rotation metadata | **none** (no `displaymatrix` side data, no rotate tag) |
| Audio | AAC LC stereo 44.1 kHz present but irrelevant (no audible sync cue verified) |
| Encoder tag | `Lavf58.45.100` — it is a re-encode, not camera-original |

**Duplicate-frame check.** `ffmpeg -vf mpdecimate=hi=64:lo=32:frac=0.1 -f null -` passed
**39 of 39** frames through — zero drops. An independent per-frame grayscale difference
confirms this: no two consecutive frames are bit-identical, and the five "idle" frames
(1-5) still differ by 59-93 changed pixels each (render noise / codec noise), not zero.

=> **39 unique animation frames at a true 24 fps.** This is already a 24 fps animation, not a
24-in-30 pulldown. Frame count matches the assumed 39 exactly.

**Video frame -> Blender frame mapping: 1:1.** Set scene fps = 24, frame_start = 1,
frame_end = 39. Video frame *n* == Blender frame *n*.
*(If you want a seamless cycle, see the loop note in section 5 — frame 39 is a duplicate of
frame 1, so a cycle should run 1..38 with 39 as the wrap copy.)*

---

## 2. What is in the shot

**Character.** A short, very round cartoon goblin/ogre: olive-green matte body, one big
egg-shaped torso with a wide brown belt/loincloth band across the hips, a large rounded-cube
head sitting directly on the torso (no visible neck), two big white eyes with black pupils,
two cream upward-pointing tusks from the lower lip, thin tube arms ending in ball fists, thin
tube legs ending in big brown flat feet. **No ears, no hair, no cloth, no tail, no fingers,
no visible cape or accessory.** Shading is a soft clay/plasticine look.

Whether this is "our" goblin I cannot judge from the video alone — I was not given the target
model to compare against. What I can say is that this reference character has **no secondary
appendages at all** (no ears, no cloth, no tail), so if the target goblin has any, the
reference gives **zero** guidance for them.

**Camera.** Static, verified objectively: over all 39 frames the maximum per-pixel deviation
of the top 40-row background strip is 0.035 and of the bottom 40-row strip 0.031 (on a 0..1
scale) — i.e. codec noise level. The larger deviations on the left and right edge columns
(0.53 / 0.50) are the club itself entering those columns at f29 and f15, not camera motion.
**No pan, no zoom, no shake, no cuts.**

**Camera angle.** Essentially **straight-on front view**, very slightly above the character's
eye line (a few degrees of downward elevation — the tops of both feet are visible and the
scalp reads 54 px above the eye line on a ~152 px-wide head). Not a 3/4 and not a side view:
at frame 1 both eyes are the same size to within 0.4%, and the eye separation (75.2 px) stays
within +/-2% of that value across all 39 frames.

**Background.** A featureless seamless studio backdrop (cyclorama), warm neutral grey, with a
soft top-to-bottom brightness gradient. No props, no set dressing, no horizon line drawn.

**Ground plane.** Yes, readable — two independent cues: (a) the backdrop brightness jumps from
~0.588 to ~0.650 between y=440 and y=504, which is the floor catching light, and (b) a soft
contact shadow pools under the feet. There is **no texture, grid, or marking** on the floor,
so exact world scale is not recoverable from the footage.

---

## 3. The weapon

| Question | Answer |
|---|---|
| What is it | A **wooden club / cudgel**. Teardrop (egg) shaped heavy head on a tapered shaft. Same brown material as the belt and the feet. |
| Size | At frame 1 it spans (42,114) to (109,358) = **253 px** end to end, against a 379 px character silhouette — the club is about **two-thirds of body height**. Head blob is ~70 px across (max inscribed radius 35 px). |
| Which hand | The **character's RIGHT hand** (appears on **screen-left** at idle). |
| One- or two-handed | **One-handed throughout.** The free hand never touches the club in any frame. |
| Grip position on shaft | Gripped low-middle: at idle the shaft protrudes ~50 px **below** the fist (a separate brown blob at bbox `[83,316,109,358]` sits below the hand). |
| Grip changes | **No.** Single, unchanging one-handed grip for all 39 frames. No hand-swap, no re-grip, no release. |

---

## 4. Phase breakdown (actual video frame numbers)

Timing is driven by two objective curves stored in `metrics.json`:
**motion energy** (mean absolute grayscale difference from the previous frame) and
**club-head displacement per frame**.

| Phase | Frames | Evidence |
|---|---|---|
| **Idle hold** | **1 – 5** | motion energy 0.0000–0.0004; only 59–93 px change per frame. Dead hold. |
| **Anticipation start** | **f6** | first real movement: energy jumps 0.0004 -> 0.0029, 3110 px change; club head first moves (1.8 px). |
| **Wind-up (raise)** | **6 – 12** | club head arcs from (78,164) up to (134,114) by f9 then over to (154,118) at f10; hips rise 354 -> 345; the character's right foot lifts and steps out. |
| **Body highest / most stretched** | **f11** | hips at their highest (y=345.0), eye-line highest (y=151.0), scalp top y=98. Head is also at its **furthest from camera** here (total eye area 0.94 of idle) — the character leans slightly *back*. |
| **Max wind-up (club fully cocked)** | **f12** | club is fully hidden behind/above the head, lying near-horizontal pointing back over the character's right shoulder (screen-left). Hips have already started to drop (345 -> 355) = the down-swing has begun. |
| **Weapon highest point** | **f13 (approximate — see caveat)** | at f13 the motion-blur streak of the club reaches **y = 51 px (norm 0.089)**, the highest brown pixel in the whole clip. The highest *cleanly resolvable, unblurred* club-head centre is **f9 at y = 113.7 (norm 0.197)**. The club is occluded behind the head at f11–f12, so the exact apex frame cannot be pinned. |
| **Swing start** | **between f13 and f14** | at f13 the club still points up-and-back (up-left of the raised hand). By f14 it has left that position entirely. |
| **Max swing speed** | **f13 -> f15, single-frame peak at f14** | club head travels from (66,102) at f13 to (395,451) at f15 = **481 px in 2 frames (~240 px/frame, ~5800 px/s)**. At f14 the club is **not visible at all** — it is so fast it is fully smeared / hidden behind the body. Motion energy also peaks here: f13 = 0.0479 (global max), f14 = 0.0461 (2nd). |
| **Weapon lowest point / impact** | **f15** | club head centre (394.7, 451.2), club bbox bottom y = 497 — the lowest brown pixel of the whole clip, 11 px below the planted foot's bottom edge, i.e. on the ground plane in front of the feet. |
| **Follow-through hold / settle** | **15 – 23** | energy collapses to 0.0019–0.0073. Club head creeps back only 23 px over 8 frames (394,451) -> (378,435): a slow settle, not a new action. Longest hold in the clip (9 frames = 0.375 s). |
| **Follow-through end / recovery start** | **f23 – f24** | energy re-accelerates 0.0073 -> 0.0126 -> 0.0214 -> 0.0308 -> 0.0421 (f27). |
| **Recovery (return swing)** | **24 – 34** | club sweeps back up the screen-left side: (364,433) f24 -> (256,411) f26 -> (55,299) f28 -> (39,250) f29 -> (62,180) f31. Recovery energy peaks at **f27** (0.0421). |
| **Idle return (settled)** | **f35** | club head returns to exactly its idle value (78.1, 163.6) at f35; eyes back to idle at f35-36. |
| **Final idle hold** | **35 – 39** | energy 0.0031 -> 0.0015 -> 0.0007 -> 0.0003 -> 0.0003. |

Note the pleasing symmetry: a 5-frame dead hold at the head (1-5) and a 5-frame dead hold at
the tail (35-39).

### Does it loop?

**Yes — frame 39 is pose-identical to frame 1.** Every tracked landmark matches to within
~0.5 px:

| Landmark | f1 | f39 |
|---|---|---|
| club head centre | (78.3, 163.6) | (78.1, 163.6) |
| hips (belt centroid) | (226.6, 353.9) | (225.7, 354.1) |
| left eye | (191.2, 162.4) | (191.1, 162.4) |
| right eye | (266.4, 162.7) | (266.5, 162.9) |
| char's right foot bbox | [110,448,193,486] | [110,446,193,487] |
| char's left foot bbox | [255,448,341,486] | [255,448,342,487] |

The residual raw-pixel difference (mean abs 0.0043) is h264 compression noise: frame 1 is an
I-frame, frame 39 is a P-frame. **Treat f39 as a duplicate of f1 and cycle on 1..38.**

---

## 5. Swing plane and direction

**It is a one-handed diagonal overhead chop that crosses the body.**

- Start of the powered arc: club head high on **screen-left** (f13 blur reaching x≈23-121,
  y≈51-181) = **over the character's RIGHT shoulder**.
- End of the powered arc: club head low on **screen-right** (f15, (395,451) norm (0.881,0.783))
  = **down and across to the character's LEFT-front**, past the character's left foot.
- Net travel: about **+330 px in x and +350 px in y** over two frames — a roughly **45°
  down-and-across diagonal on screen**.
- Rotationally: on screen the club head goes **left side up -> over the top -> right side
  down**, i.e. **clockwise as seen by the camera**.
- Character-relative: **from over the right shoulder, down across the chest, finishing
  outside the left knee.** A cross-body diagonal chop, not a pure vertical chop and not a
  horizontal sweep.

**Swing plane depth.** Mostly in the camera-facing (frontal) plane, tipped slightly toward the
camera at the end. Evidence: the hips-to-head distance barely shortens (191.3 px at f1 vs
186.5 px at f15, -2.5%), which rules out a big forward/backward plane rotation; but the eye
blobs grow to 1.19x idle area at f18-f22, so the head (and the swing's end) ends up roughly
8-9% closer to the camera than idle. So: a frontal-plane diagonal with a modest lean into
camera on the follow-through.

**Does the club hit the ground?** It **reaches ground level**. At f15-f16 the club-head bbox
bottom is y = 495-497, which is 9-11 px *below* the bottom of the planted foot and sits in the
floor's brightening band, and it is forward of the feet so the ground there is lower on
screen. It looks like it comes to rest on or a hair above the floor.
**There is no impact event staged**: no dust, no ground crack, no squash on contact, no
rebound/bounce, no stop-frame. The club simply arrives and eases to a stop over f15-f23.
I can see no separate contact shadow under the club head distinct from the body shadow, so I
cannot prove hard contact — treat "grazes/rests on the ground" as the honest reading.

---

## 6. Feet

Objective: per-frame bounding boxes of the two brown foot blobs.

| Foot | Behaviour |
|---|---|
| **Character's LEFT foot** (screen-RIGHT, bbox x≈255-342) | **Planted for all 39 frames. It never moves.** bbox is `[255,448,341,486]` at f1 and `[255,448,342,487]` at f39; between f1 and f39 it varies by at most 1-2 px (anti-aliasing). No lift, no pivot, no slide. |
| **Character's RIGHT foot** (screen-LEFT, bbox x≈110-193) | Steps out. See table below. |

Character's right foot, frame by frame:

| Frames | State | Evidence (bbox) |
|---|---|---|
| 1 – 9 | planted, idle position | `[110,448,193,486]`, identical every frame |
| **10** | starts to lift | `[108,444,191,485]` — top edge rises 4 px |
| **11** | **airborne (peak)** | `[96,432,175,475]` — top edge 16 px higher, whole foot 16 px further screen-left |
| 12 | coming down | `[95,436,172,487]` |
| **13** | **lands** in the wide stance | `[92,447,169,493]` |
| 14 – 26 | planted, wide stance | `[91,448,167,494]`, rock solid for 13 frames |
| **27 – 31** | **slides back** to idle position — this is a SLIDE, not a step | bottom edge never rises above y=446, so it stays in ground contact while x-centre travels 129 -> 149.5 |
| 32 – 39 | planted, idle position | `[110,446,193,487]` |

Summary: **one foot lift (f10-f13, a single 22 px step outward to the character's right / screen-left,
plus ~8 px toward camera), and one foot slide (f27-f31) on the way back.** No pivot is
detectable — the foot blobs keep the same shape and orientation throughout, so any pivot is
under a few degrees. **The foot slide on the return is a cartoon cheat and will read as a
slide in Blender too if you copy it literally.**

---

## 7. Body

All values below are from `metrics.json`. "Hips" = the centroid of the brown belt blob.
"Lean" = the angle of the hips -> eye-midpoint vector away from screen vertical, positive =
top tilted toward screen-right (the character's left).

### Hips / centre of gravity

| Phase | Hips (px) | Change vs idle |
|---|---|---|
| Idle f1 | (226.6, 353.9) | — |
| Top of wind-up f11 | (231.9, 345.0) | **up 9 px**, screen-right 5 px |
| Launch f13 | (234.8, 376.0) | down 22 px (already dropping) |
| Impact f15 | (209.1, 393.2) | **down 39 px from idle / 48 px from the top**, **screen-LEFT 18 px** |
| Deepest f24 | (195.2, 386.3) | screen-LEFT **31 px** from idle |
| f31 | (226.1, 357.5) | back |
| Idle f39 | (225.7, 354.1) | back to start |

So the COG does the classic **rise (9 px) -> big drop (48 px total, ~12.6% of the 379 px
character height)**. Laterally the hips go the **opposite way to the swing**: the club goes
screen-right, the hips slide screen-left by up to 31 px. That counter-shift is the single most
important body note for matching this reference.

### Crouch / rise

A pure crouch curve is *not* the silhouette bbox height (the club contaminates it).
Use instead the topmost **green** body pixel, `green_body_top_y` in `metrics.json`:
108 (idle) -> **98 at f11 (rises 10 px)** -> 140 at f15 (drops 42 px, the deepest crouch) ->
back to 108 by f31.

### Chest / spine lean (screen-space)

| Frame | 1 | 9 | 11 | 12 | 13 | 14 | **15** | 18 | **24** | 27 | 29 | 31 | 39 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| lean (deg) | 0.7 | 0.9 | 2.2 | 3.0 | 9.8 | 15.3 | **25.1** | 25.2 | **27.1** | 19.7 | 3.9 | 1.1 | 0.9 |

Near-vertical through the whole wind-up, then it dumps **25-27° toward screen-right (the
character's left) in only 3 frames (f12 -> f15)**, holds ~25° for the whole follow-through,
and unwinds over f24 -> f31. Maximum lean is actually at **f24**, i.e. it keeps *creeping* a
couple of degrees further during the hold before recovering — a nice settle.

### Chest rotation (twist)

The character does twist toward its own left (screen-right), but only modestly and I cannot
give a reliable number. Evidence: the belt blob narrows from 209 px wide at f1 to 154-162 px
at f15-f21 while getting taller (55 -> 73-75 px) — consistent with the torso rotating and
tipping. That narrowing is confounded by the arm and leg occluding the belt, so **treat the
twist angle as unmeasured** (somewhere in the 20-40° band, direction certain: **toward the
character's LEFT**).

### Head

- **Yaw:** slight, toward the character's LEFT (face turns toward screen-right). Measured via
  eye-disc areas: symmetric (ratio 0.99-1.02) through f1-f12, then the screen-LEFT eye (=the
  character's RIGHT eye, the one swinging toward camera) grows to **1.30-1.36x** the other at
  f15-f24. Projected eye separation stays at 74.5-75.7 px throughout, so the yaw is modest —
  I'd put it in the 20-35° range but **cannot pin it precisely** from a single view.
- **Pitch:** the head **looks down**. Scalp visible above the eye line goes 54.6 px (idle) ->
  **84.2 px at f15** -> holds ~80 px through f24 -> back to 54.6 by f35. On a ~152 px-wide
  head that is roughly a **25-30° downward pitch**, peaking exactly at impact (f15).
- **Roll:** rolls clockwise on screen (toward screen-right / the character's left):
  0.2° at idle -> 4.1° f12 -> 8.3° f13 -> **peaks 10.4° at f22** -> back to 0.4° by f31.
- **Depth:** the head pulls *away* from camera during the wind-up (total eye area drops to
  0.94x at f10) and comes *toward* camera on the follow-through (1.19-1.20x at f18-f22).
- The head leads the swing: eye-midpoint x goes 228.8 (f1) -> 288.2 (f15), +59 px screen-right.
- **No blinks.** The two white eye discs are present and the same size family in every one of
  the 39 frames; the pupils never disappear.

### Free arm (character's LEFT arm, screen-right)

It is a pure counterweight and it is very active:

| Frame | free hand (norm) | note |
|---|---|---|
| 1 | (0.855, 0.565) | hanging by the hip |
| 9 | (0.905, 0.465) | swung out and up |
| **12** | (0.905, 0.270) | **highest — thrown up, mirroring the cocked club** |
| 13 | (0.885, 0.325) | |
| 15 – 24 | ~(0.825-0.840, 0.395-0.400) | **folded in and held up beside the head** for the whole follow-through |
| 29 | (0.890, 0.520) | dropping |
| 39 | (0.855, 0.565) | back to idle |

(These free-hand values are read off the 0.1 grid overlays by eye, ±0.02, not auto-tracked.)

### Squash & stretch / cartoon exaggeration

**No volumetric squash & stretch is detectable.** Checks that would have shown it:
- eye separation constant (74.5-77.4 px, ±2%) => head is never squashed or stretched laterally;
- hips-to-head-top vertical extent: 246 px at f1, 247 px at f11, 253 px at f15, 254 px at f21
  => the torso does not compress on impact nor elongate on the stretch;
- the belt band keeps its thickness.

The exaggeration in this clip is entirely **timing, arcs, motion blur and pose extremes**:
the 2-frame swing, the 9-frame dead hold afterwards, the deep 48 px COG drop, the 25° lean,
the 31 px counter-shift of the hips.

### Secondary motion

**None.** The character has no ears, hair, cloth, belt tails, tail, or props that could
overlap. The only overlapping-action-like behaviour is the **slow settle over f15-f23** (club
head drifts 23 px, hips drift 7 px, head roll creeps from 8.0° to 10.4°) and the **2° extra
lean creep to f24** — i.e. easing, not physics.

---

## 8. Key-pose table

Normalized screen coordinates, origin top-left, x/448 and y/576.

**Source of each column:**
- *club head centre* and *club tip* — automatic (distance-transform on the segmented brown
  club blob), objective;
- *hips* — automatic (belt blob centroid), objective;
- *head centre* — automatic (midpoint of the two eye discs), objective; this is the **eye
  line**, roughly the middle of the face, not the geometric centre of the head volume;
- *left / right foot* — automatic (foot blob bbox: x = horizontal centre, y = bottom edge,
  i.e. the ground contact point);
- *grip / hand on club* — **read by eye off the 0.1 grid overlays in `ref/keyframes/`,
  ±0.02**;
- *chest tilt* — automatic (hips -> eye-line angle from screen vertical, + = top toward
  screen-right = the character's left).

| f | pose | club tip | club head ctr | grip (hand) | head ctr (eyes) | hips | char-RIGHT foot (screen-L) | char-LEFT foot (screen-R) | chest tilt |
|---|---|---|---|---|---|---|---|---|---|
| **1** | Idle. Weight even, both feet planted, club held vertical head-up beside the character's right hip, free arm hanging. | 0.167, 0.241 | 0.175, 0.284 | 0.215, 0.495 | 0.511, 0.282 | 0.506, 0.614 | 0.338, 0.844 | 0.665, 0.844 | **0.7°** |
| **9** | Anticipation extreme (raise). Club swung up and inboard, head of the club above the shoulder; body stretched tall (hips at their highest region), head leaning very slightly back from camera; free arm swung out. | 0.321, 0.165 | 0.298, 0.197 | 0.145, 0.325 | 0.518, 0.274 | 0.512, 0.602 | 0.338, 0.844 | 0.665, 0.844 | 0.9° |
| **12** | **Max wind-up.** Club fully cocked back over the character's right shoulder, lying near-horizontal, mostly hidden behind the head. Free arm thrown high (its highest frame). Right foot has just landed wide. Hips already starting to drop. | 0.368, 0.255 * | 0.355, 0.251 * | 0.285, 0.245 | 0.546, 0.277 | 0.523, 0.617 | 0.298, 0.845 | 0.665, 0.844 | 3.0° |
| **13** | **Swing launch / top of arc.** Arm fully extended up-and-back, club a long motion-blur streak reaching the top of frame (highest brown pixel of the clip, y=0.089). Hips have dropped 22 px. | 0.118, 0.139 † | 0.148, 0.178 † | 0.295, 0.400 | 0.598, 0.322 | 0.524, 0.653 | 0.291, 0.856 | 0.665, 0.844 | 9.8° |
| **(14)** | **Fastest frame — club invisible.** Body already crouched and rotated; club is fully smeared / occluded. No club measurement possible. | — | — | — | 0.628, 0.356 | (0.524, 0.653) ‡ | 0.288, 0.858 | 0.665, 0.844 | 15.3° |
| **15** | **Impact / lowest point.** Club head on the ground outside the character's left foot. Deepest crouch, arm crossed all the way over the body, head pitched down ~30° and rolled 8°, hips shifted screen-left. | 0.922, 0.806 | 0.881, 0.783 | 0.725, 0.660 | 0.643, 0.389 | 0.467, 0.683 | 0.288, 0.858 | 0.665, 0.844 | **25.1°** |
| **18** | Follow-through hold, mid-settle. Essentially the f15 pose easing back by a few pixels. | 0.902, 0.799 | 0.868, 0.774 | 0.715, 0.655 | 0.643, 0.380 | 0.462, 0.680 | 0.288, 0.858 | 0.666, 0.844 | 25.2° |
| **24** | **Follow-through extreme / recovery start.** Maximum lean (27.1°) and maximum hip counter-shift (31 px screen-left). Club has lifted clear of the ground. | 0.850, 0.766 | 0.812, 0.751 | 0.665, 0.655 | 0.635, 0.368 | 0.436, 0.671 | 0.288, 0.858 | merged ‡ | **27.1°** |
| **29** | Recovery mid. Club has swung back up the screen-left side and is near-horizontal pointing up-left; body is almost vertical again; right foot mid-slide back to idle. | 0.045, 0.418 | 0.088, 0.433 | 0.220, 0.530 | 0.523, 0.314 | 0.495, 0.637 | 0.312, 0.851 | 0.666, 0.845 | 3.9° |
| **33** | Settle. Club 2 px short of the idle position, body already there. | 0.156, 0.248 | 0.166, 0.289 | 0.215, 0.500 | 0.511, 0.283 | 0.504, 0.615 | 0.338, 0.845 | 0.666, 0.845 | 1.0° |
| **39** | Final idle — identical to f1. | 0.167, 0.241 | 0.174, 0.284 | 0.215, 0.495 | 0.511, 0.282 | 0.504, 0.615 | 0.338, 0.845 | 0.666, 0.845 | 0.9° |

\* f12: the club is ~75% hidden behind the head. Only a sliver was segmentable (blob area
1.6-2.0 k px vs 7.2 k at idle), so the f12 club values are the **visible sliver**, not the true
club-head centre. The true club head at f12 is **behind the head**, further screen-left.

† f13: the club is heavily motion-blurred. The reported point is the centroid of the thickest
part of the blur streak, which is a time-average over the shutter, not an instantaneous pose.

‡ f14 / f24-f26: the belt blob (f14, f25, f26) or the foot blob (f24-f26) merged with the club
in the colour segmentation. Values in parentheses are carried over from the previous frame and
are flagged `hips_estimated: true` in `metrics.json`.

---

## 9. Objective curves (all in `ref/metrics.json`)

Per frame, `metrics.json` contains:

- `motion_energy` — mean abs grayscale difference vs the previous frame. **This is the timing
  curve.** Peaks: f13 = 0.0479, f14 = 0.0461, f27 = 0.0421 (recovery). Floors: f1-5 ≈ 0.0004,
  f20-22 ≈ 0.002, f37-39 ≈ 0.0005.
- `club.head_center`, `club.head_outer_tip`, `club.bbox`, `club_head_speed_px`,
  `club_tip_speed_px`, plus `merged_with_foot` / `occluded` honesty flags.
- `hips_belt_centroid` (+ `_filled` and `hips_estimated`).
- `eyes`, `eye_blobs` (x, y, pixel area, width), `eye_mid`, `eye_sep_px`, `head_roll_deg`,
  `eye_area_ratio_L_over_R`, `eye_total_area_vs_f1`.
- `spine_lean_deg_screen`, `hips_to_head_px`.
- `green_body_top_y`, `scalp_above_eyes_px`, `head_pitch_proxy_px`.
- `silhouette_bbox` / `_h` / `_w` / `_centroid`, `feet_bboxes`.
- `loop_check`.

Condensed timing curve:

| f | 1 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | **13** | **14** | 15 | 16-22 | 23 | 24 | 25 | 26 | **27** | 28 | 29 | 30 | 31 | 32 | 33 | 35 | 39 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| energy (x1000) | 0.0 | 2.9 | 13.9 | 23.8 | 27.5 | 27.6 | 33.3 | 23.6 | **47.9** | **46.1** | 38.2 | 2-7 | 7.3 | 12.6 | 21.4 | 30.8 | **42.1** | 31.1 | 27.6 | 25.5 | 34.2 | 12.5 | 6.6 | 3.1 | 0.3 |

---

## 10. Files produced

All under `C:\Users\whxod\orca\anime\work\goblin_swing\ref\`:

| Path | What |
|---|---|
| `reference.mp4` | ASCII-named copy of the source video |
| `frames\f_0001.png` … `f_0039.png` | all 39 frames, full resolution 448x576, 1-based |
| `sheets\sheet_A.png` | contact sheet, frames 1-20, 5 cols, frame numbers burned in |
| `sheets\sheet_B.png` | contact sheet, frames 21-39, 5 cols, frame numbers burned in |
| `sheets\debug_club_A.png`, `debug_club_B.png` | the same sheets with the auto-tracked club bbox (cyan), club far-point (red) and body centroid (green) drawn on — use these to sanity-check the tracking |
| `keyframes\key_0001.png` … `key_0039.png` (10 files: 1, 9, 12, 13, 15, 18, 24, 29, 33, 39) | 2x upscaled key frames with a 0.1 normalized coordinate grid burned in, plus markers for eye-midpoint (yellow), hips (green), club head (red) |
| `zoom\z_*.png` | 2x upscaled clean frames for the 17 frames inspected by eye |
| `metrics.json` | all per-frame objective measurements described in section 9 |
| `club_track.json` | intermediate club-tracking pass (superseded by `metrics.json`) |
| `REFERENCE_ANALYSIS.md` | this document |

---

## 11. UNCERTAINTIES — things I could NOT determine from this footage

1. **Is this the same goblin as our target character?** I was not given the target model, so I
   cannot compare. The reference character has no ears, no cloth, no tail, no fingers.
2. **Exact club apex frame.** The club is occluded behind the head at **f11-f12** and heavily
   motion-blurred at **f13**. The highest resolvable unblurred club-head centre is f9; the
   highest brown pixel of the clip is inside the f13 blur. The true apex is somewhere in
   f11-f13 and cannot be pinned to a frame.
3. **Frame 14 is unmeasurable for the weapon.** The club is completely smeared / occluded. Its
   position at f14 is an interpolation between f13 and f15, not an observation.
4. **Torso twist (yaw) angle.** Direction is certain (toward the character's LEFT), magnitude
   is not: the belt blob is partly occluded by the arm and leg during the follow-through, so
   the 209 -> 154 px narrowing conflates twist with occlusion and with forward tip. Estimate
   20-40°, unverified.
5. **Head yaw angle.** Same problem from the other direction: eye areas prove a yaw toward the
   character's left exists (ratio up to 1.36) but a single front view plus unknown camera FOV
   cannot resolve it to a number. Estimate 20-35°, unverified.
6. **Whether the club actually touches the ground at f15-f22.** It reaches ground level, but I
   can see no separate contact shadow under the club head, no dust, no squash, no rebound, and
   no ground deformation. "Resting on or just above the floor" is the honest reading.
7. **Any 3D depth at all.** Single static front camera, no parallax, no floor texture, no
   reference object of known size => **no world-space measurements, no absolute scale, no depth
   for any joint.** Every number in this document is a screen-space projection. In particular,
   motion toward/away from camera is only inferable from the eye-disc area proxy, which is
   coarse.
8. **Foot pivot / roll.** The foot blobs keep the same silhouette shape throughout, so any
   ankle pivot or toe roll is below the detectable threshold from this view (a few degrees). I
   cannot say whether there is a heel or toe pivot.
9. **The free (left) hand coordinates in the key-pose table are eyeballed**, not auto-tracked —
   the hand is the same green as the body, so colour segmentation cannot isolate it. ±0.02.
10. **Head "centre" is the eye-line midpoint**, not the volumetric centre of the head. If you
    need the head bone's pivot, offset accordingly.
11. **Elbow, shoulder, knee and wrist positions were not measured at all** — the limbs are the
    same green as the torso and there is no reliable way to segment them from this footage.
12. **No audio cue was used.** There is an AAC track but I did not verify whether it contains a
    usable impact transient, so nothing in the timing above depends on it.
13. **Interpolation / spacing within the swing.** With only two observations (f13, f15)
    bracketing the fastest part of the move, I cannot recover the actual spacing curve inside
    the swing — only that the whole ~480 px of travel happens inside those 2 frames.
