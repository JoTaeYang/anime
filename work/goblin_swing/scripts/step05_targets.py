"""STEP 05 target data.

Everything the blocking solver needs that is NOT already in ref/metrics.json.
Screen coords are px at 448x576, origin top-left (same convention as metrics.json
and render_compare.py).

Landmark offsets applied when a metrics.json value is turned into a target for
OUR rig (documented in render_compare.py's header, all measured at frame 1):
    hips        target = metrics hips_belt_centroid + (-1.3, +20.2)
    weapon_tip  target = metrics club.head_outer_tip + (0, -18.4)
    club head   target = metrics club.head_center            (blobfit point, no offset)
    foot_L/R    target = (bbox x-centre, bbox bottom) + (+0.81, -2.89)   [structural]
    hands       the reference hand coords are eyeballed; we remove the f1 bias
                hand_R += (-5.06, +7.14)   hand_L += (-9.78, +7.75)
                (= our rest projection minus the f1 table value)

Authored (non-measurable) values carry a `src` tag: "meas" | "eye" | "authored".
"""

KEYS = [1, 6, 9, 11, 12, 13, 15, 18, 24, 27, 29, 31, 33, 35, 39]
IDLE = [1, 6, 35, 39]          # exact rest pose, zero controls

FOOT_CAL = (0.81, -2.89)
HIPS_CAL = (-1.3, 20.2)
TIP_CAL = (0.0, -18.4)
HANDR_CAL = (-5.06, 7.14)
HANDL_CAL = (-9.78, 7.75)

PXM = 197.0    # px per metre near the character (hips->head_top calibration)

# ---------------------------------------------------------------- right hand (grip)
# from REFERENCE_ANALYSIS section 8 "grip (hand)" column (normalised, eyeballed +-0.02)
GRIP_NORM = {1: (0.215, 0.495), 6: (0.215, 0.495), 35: (0.215, 0.495),
             9: (0.145, 0.325), 12: (0.285, 0.245),
             13: (0.295, 0.400), 15: (0.725, 0.660), 18: (0.715, 0.655),
             24: (0.665, 0.655), 29: (0.220, 0.530), 33: (0.215, 0.500),
             39: (0.215, 0.495)}
# frames with no table row: read off the zoom frames by eye, already in px, no calib
GRIP_PX_AUTH = {11: (120.0, 140.0),     # 40 px along the club from the butt knob
                27: (120.0, 320.0),     # fist in z_0027
                31: (91.7, 298.0)}      # interpolated toward idle

# ---------------------------------------------------------------- left (free) hand
HANDL_NORM = {1: (0.855, 0.565), 6: (0.855, 0.565), 35: (0.855, 0.565),
              9: (0.905, 0.465), 12: (0.905, 0.270),
              13: (0.885, 0.325), 15: (0.832, 0.398), 18: (0.832, 0.398),
              24: (0.832, 0.397), 29: (0.890, 0.520), 33: (0.855, 0.560),
              39: (0.855, 0.565)}
HANDL_PX_AUTH = {11: (395.7, 203.6),    # between f9 and f12, "thrown up/out high"
                 27: (405.0, 267.0),    # read off z_0027 (arm out sideways)
                 31: (380.0, 321.7)}

# ---------------------------------------------------------------- club
# club head centre / outer tip.  None = not usable from the footage.
# w = weight multiplier for this frame's club residuals.
CLUB = {
    1:  dict(head=(78.3, 163.6),   tip=(75.0, 139.0),  w=1.0, src="meas"),
    6:  dict(head=(77.8, 161.9),   tip=(74.0, 138.0),  w=1.0, src="meas"),
    9:  dict(head=(133.5, 113.7),  tip=(144.0, 95.0),  w=1.0, src="meas"),
    # f11/f12: club head hidden BEHIND the goblin's head.  What is measurable is the
    # shaft BUTT knob sticking out up-screen-left, and the fact that the club is
    # near-horizontal and strongly foreshortened (pointing away from camera).
    11: dict(head=(170.0, 145.0), tip=None, w=0.35, src="authored",
             butt=(78.0, 120.0), butt_w=2.0, depth=0.45),
    12: dict(head=(185.0, 152.0), tip=None, w=0.35, src="authored",
             butt=(93.0, 122.0), butt_w=2.0, depth=0.50),
    13: dict(head=(66.1, 102.3),   tip=(53.0, 80.0),   w=0.7, src="meas(blurred)"),
    15: dict(head=(394.7, 451.2),  tip=(413.0, 464.0), w=1.0, src="meas"),
    18: dict(head=(388.7, 445.7),  tip=(404.0, 460.0), w=1.0, src="meas"),
    24: dict(head=(363.9, 432.6),  tip=(381.0, 441.0), w=0.8, src="meas(merged)"),
    27: dict(head=(97.0, 362.0),   tip=None,           w=0.4, src="authored",
             depth=-0.35),
    29: dict(head=(39.3, 249.6),   tip=(20.0, 241.0),  w=1.0, src="meas"),
    31: dict(head=(62.4, 180.4),   tip=(57.0, 156.0),  w=1.0, src="meas"),
    33: dict(head=(74.5, 166.7),   tip=(70.0, 143.0),  w=1.0, src="meas"),
    35: dict(head=(78.1, 163.6),   tip=(75.0, 139.0),  w=1.0, src="meas"),
    39: dict(head=(78.1, 163.6),   tip=(75.0, 139.0),  w=1.0, src="meas"),
}

# world-Y of the club head (depth).  -Y = toward camera.  Occlusion order evidence:
#   f11/f12 club BEHIND the head, f15-f24 club+arm in FRONT of the belly.
CLUB_DEPTH = {9: 0.00, 11: 0.45, 12: 0.50, 13: 0.00,
              15: -0.55, 18: -0.55, 24: -0.50,
              27: -0.35, 29: -0.30, 31: -0.32, 33: -0.33}

# world-Y of the right grip.  Rest = -0.234.  f11/f12 the hand is up beside the head
# (roughly the body plane); f15-f24 it is out in front of the belly.
HAND_DEPTH = {9: -0.30, 11: 0.05, 12: 0.10, 13: -0.30,
              15: -0.45, 18: -0.45, 24: -0.45,
              27: -0.30, 29: -0.25, 31: -0.24, 33: -0.23}

# elbow flexion offset from REST, degrees (rest R flexion ~44 deg, L ~30 deg).
# -40 = arm straight, +60..+80 = tightly folded.  Read off the reference frames;
# used as a weak prior so the optimiser picks a natural elbow among equal solutions.
ELBOW_R_PRIOR = {9: 40, 11: 60, 12: 65, 13: -35, 15: -20, 18: -20, 24: -15,
                 27: -10, 29: 10, 31: 0, 33: 0}
ELBOW_L_PRIOR = {9: -10, 11: -10, 12: -10, 13: -10, 15: 60, 18: 60, 24: 60,
                 27: -10, 29: -5, 31: 0, 33: 0}

# right ELBOW (FOREARM_R bone head) screen position, read off the reference frames
# by eye (the limbs are the same green as the torso so this cannot be segmented).
# Low weight - it is only there to stop the solver routing the arm behind the body.
ELBOW_R_PX = {9: (58.0, 222.0), 11: (105.0, 185.0), 12: (112.0, 190.0),
              13: (175.0, 255.0),
              # f15-f24 the upper arm is hidden behind the torso and the forearm
              # crosses IN FRONT of the belly; the elbow reads just under the chin.
              15: (215.0, 292.0), 18: (213.0, 288.0), 24: (205.0, 285.0),
              27: (125.0, 300.0),
              29: (100.0, 292.0), 31: (96.0, 292.0), 33: (93.0, 292.0)}
ELBOW_R_PX_W = 1.5

# ---------------------------------------------------------------- right foot
# z = FOOT_IK_R world lift in metres (0 = sole flat on the floor).
# y-target override (px) when the foot is authored as airborne.
FOOT_R = {
    1:  dict(z=0.0), 6: dict(z=0.0), 9: dict(z=0.0),
    11: dict(z=0.056),                                  # ref: bbox top 16 px higher
    12: dict(z=0.0),
    13: dict(z=0.0), 15: dict(z=0.0), 18: dict(z=0.0), 24: dict(z=0.0),
    27: dict(z=0.0),
    # USER DECISION: the f27-f31 return is a STEP, not a slide -> lift it at f29.
    29: dict(z=0.055, px=(140.8, 476.0), src="authored (step, not the ref slide)"),
    31: dict(z=0.0), 33: dict(z=0.0), 35: dict(z=0.0), 39: dict(z=0.0),
}

# ---------------------------------------------------------------- head yaw prior
# unmeasurable from a single front view (REFERENCE_ANALYSIS 11.5 -> 20-35 deg,
# direction certain: toward the character's LEFT = +Z rotation).
HEAD_YAW_PRIOR = {1: 0, 6: 0, 9: 0, 11: 0, 12: 2, 13: 10, 15: 25, 18: 28,
                  24: 28, 27: 12, 29: 3, 31: 0, 33: 0, 35: 0, 39: 0}
# torso twist prior (COG+HIPS+CHEST yaw share), same caveat (20-40 deg est.)
TORSO_YAW_PRIOR = {1: 0, 6: 0, 9: 0, 11: 0, 12: 3, 13: 12, 15: 32, 18: 34,
                   24: 34, 27: 14, 29: 3, 31: 0, 33: 0, 35: 0, 39: 0}

# ---------------------------------------------------------------- manual body seeds
# (cog_loc m, cog_rot deg, hips_rot deg, chest_rot deg, head_rot deg) world axes
# +X rot = pitch forward/look down, +Y rot = top toward screen-RIGHT, +Z = yaw to
# the character's LEFT.  Only used as an optimiser seed; the solver refines them.
BODY_SEED = {
    9:  ((0.00, 0.0, 0.030), (0, 0, 0), (0, 0, 0), (0, 1, 0), (-1, 0, 0)),
    11: ((-0.01, 0.02, 0.045), (0, 1, 0), (0, 1, 0), (0, 1, 0), (-2, 0, 0)),
    12: ((-0.01, 0.01, -0.010), (1, 1, 2), (0, 1, 0), (1, 1, 1), (4, 2, 2)),
    13: ((-0.01, -0.02, -0.110), (4, 6, 6), (1, 2, 2), (2, 2, 2), (8, 2, 8)),
    15: ((-0.09, -0.05, -0.150), (8, 16, 15), (2, 4, 5), (3, 5, 6), (20, 0, 22)),
    18: ((-0.10, -0.06, -0.145), (8, 16, 15), (2, 4, 5), (3, 5, 6), (18, 0, 24)),
    24: ((-0.15, -0.05, -0.135), (8, 18, 15), (2, 4, 5), (3, 5, 6), (17, 0, 24)),
    27: ((-0.20, -0.03, -0.060), (5, 12, 7), (1, 3, 3), (2, 4, 3), (14, 0, 10)),
    29: ((-0.02, -0.02, -0.040), (1, 2, 1), (0, 1, 1), (1, 1, 1), (11, 0, 2)),
    31: ((0.00, -0.01, -0.010), (0, 1, 0), (0, 0, 0), (0, 0, 0), (4, 0, 0)),
    33: ((0.00, 0.0, -0.002), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)),
}


def grip_px(f):
    if f in GRIP_PX_AUTH:
        return GRIP_PX_AUTH[f], "authored"
    n = GRIP_NORM[f]
    return (n[0] * 448 + HANDR_CAL[0], n[1] * 576 + HANDR_CAL[1]), "eye"


def handl_px(f):
    if f in HANDL_PX_AUTH:
        return HANDL_PX_AUTH[f], "authored"
    n = HANDL_NORM[f]
    return (n[0] * 448 + HANDL_CAL[0], n[1] * 576 + HANDL_CAL[1]), "eye"
