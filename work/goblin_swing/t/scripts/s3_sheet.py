"""Build the GATE 03 contact sheets (labelled) from the rendered PNGs."""
import bpy, os, sys, json
import numpy as np

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
SRC = os.path.join(T, "inspect", "step03", "hiF")
DST = os.path.join(T, "inspect", "step03")

F = {
 ' ':"00000,00000,00000,00000,00000,00000,00000", '_':"00000,00000,00000,00000,00000,00000,11111",
 '-':"00000,00000,00000,11111,00000,00000,00000", '.':"00000,00000,00000,00000,00000,01100,01100",
 '+':"00000,00100,00100,11111,00100,00100,00000", '/':"00001,00010,00100,00100,01000,10000,00000",
 ':':"00000,01100,01100,00000,01100,01100,00000",
 '0':"01110,10001,10011,10101,11001,10001,01110",'1':"00100,01100,00100,00100,00100,00100,01110",
 '2':"01110,10001,00001,00010,00100,01000,11111",'3':"11111,00010,00100,00010,00001,10001,01110",
 '4':"00010,00110,01010,10010,11111,00010,00010",'5':"11111,10000,11110,00001,00001,10001,01110",
 '6':"00110,01000,10000,11110,10001,10001,01110",'7':"11111,00001,00010,00100,01000,01000,01000",
 '8':"01110,10001,10001,01110,10001,10001,01110",'9':"01110,10001,10001,01111,00001,00010,01100",
 'A':"01110,10001,10001,11111,10001,10001,10001",'B':"11110,10001,10001,11110,10001,10001,11110",
 'C':"01110,10001,10000,10000,10000,10001,01110",'D':"11100,10010,10001,10001,10001,10010,11100",
 'E':"11111,10000,10000,11110,10000,10000,11111",'F':"11111,10000,10000,11110,10000,10000,10000",
 'G':"01110,10001,10000,10111,10001,10001,01111",'H':"10001,10001,10001,11111,10001,10001,10001",
 'I':"01110,00100,00100,00100,00100,00100,01110",'J':"00111,00010,00010,00010,00010,10010,01100",
 'K':"10001,10010,10100,11000,10100,10010,10001",'L':"10000,10000,10000,10000,10000,10000,11111",
 'M':"10001,11011,10101,10101,10001,10001,10001",'N':"10001,10001,11001,10101,10011,10001,10001",
 'O':"01110,10001,10001,10001,10001,10001,01110",'P':"11110,10001,10001,11110,10000,10000,10000",
 'Q':"01110,10001,10001,10001,10101,10010,01101",'R':"11110,10001,10001,11110,10100,10010,10001",
 'S':"01111,10000,10000,01110,00001,00001,11110",'T':"11111,00100,00100,00100,00100,00100,00100",
 'U':"10001,10001,10001,10001,10001,10001,01110",'V':"10001,10001,10001,10001,10001,01010,00100",
 'W':"10001,10001,10001,10101,10101,11011,10001",'X':"10001,10001,01010,00100,01010,10001,10001",
 'Y':"10001,10001,01010,00100,00100,00100,00100",'Z':"11111,00001,00010,00100,01000,10000,11111",
}
GL = {k: np.array([[int(c) for c in r] for r in v.split(",")], bool) for k, v in F.items()}


def text(img, x, y, s, scale=3, col=(0.05, 0.05, 0.05)):
    s = s.upper()
    for ch in s:
        g = GL.get(ch, GL[' '])
        for r in range(7):
            for c in range(5):
                if g[r, c]:
                    img[y + r * scale:y + (r + 1) * scale, x + c * scale:x + (c + 1) * scale] = col
        x += 6 * scale
    return x


def load(p):
    im = bpy.data.images.load(p)
    w, h = im.size
    a = np.array(im.pixels[:], dtype=np.float32).reshape(h, w, 4)[::-1, :, :3]
    bpy.data.images.remove(im)
    return a


def fit(a, tw, th):
    h, w, _ = a.shape
    ys = (np.arange(th) * (h / th)).astype(int).clip(0, h - 1)
    xs = (np.arange(tw) * (w / tw)).astype(int).clip(0, w - 1)
    return a[ys][:, xs]


def sheet(items, cols, cw, ch, out, title, lab_h=34):
    rows = (len(items) + cols - 1) // cols
    W, H = cols * cw, rows * (ch + lab_h) + 46
    img = np.ones((H, W, 3), np.float32) * 0.93
    text(img, 12, 12, title, 3)
    for i, (path, label) in enumerate(items):
        r, c = divmod(i, cols)
        x0, y0 = c * cw, 46 + r * (ch + lab_h)
        if os.path.exists(path):
            img[y0:y0 + ch, x0:x0 + cw] = fit(load(path), cw, ch)
        else:
            img[y0:y0 + ch, x0:x0 + cw] = 0.8
        img[y0 + ch:y0 + ch + lab_h, x0:x0 + cw] = 0.85
        text(img, x0 + 8, y0 + ch + 9, label[:int(cw / 18)], 3)
        img[y0:y0 + ch + lab_h, x0:x0 + 2] = 0.6
        img[y0:y0 + 2, x0:x0 + cw] = 0.6
    o = bpy.data.images.new("sheet", W, H, alpha=False)
    px = np.ones((H, W, 4), np.float32)
    px[:, :, :3] = img[::-1]
    o.pixels = px.ravel()
    o.filepath_raw = out
    o.file_format = 'PNG'
    o.save()
    print("WROTE", out, W, "x", H)


POSES = [("A_rest", "A rest T-pose"), ("A_arms_down", "A arms-down"), ("A_idle", "A idle"),
         ("A_windup", "A wind-up"), ("A_impact", "A impact"),
         ("B_f1", "B f1 idle"), ("B_f11", "B f11 wind-up"), ("B_f15", "B f15 impact"),
         ("B_f29", "B f29 recovery")]
sheet([(os.path.join(SRC, "g_%s_ref.png" % k), v) for k, v in POSES], 3, 336, 432,
      os.path.join(DST, "gate03_sheet_hi.png"),
      "GATE 03  high-poly  REF_CAM  pose sets A+B")

CU = [("shR_front", "R shoulder front"), ("shR_back", "R shoulder back"),
      ("shR_low", "R armpit low"), ("elbR", "R elbow"),
      ("wrR", "R wrist/fist"), ("legs", "leg roots")]
items = []
for pk, pn in (("A_windup", "windup"), ("A_impact", "impact")):
    for ck, cn in CU:
        items.append((os.path.join(SRC, "g_%s_%s.png" % (pk, ck)), "%s %s" % (pn, cn)))
sheet(items, 6, 300, 300, os.path.join(DST, "gate03_sheet_closeups_hi.png"),
      "GATE 03  high-poly close-ups  wind-up (top) / impact (bottom)")
