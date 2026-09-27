"""Pixel tracking of the death reference (main measurement, report-only). Output: measure.json"""
import json, numpy as np
from PIL import Image
def seg(im):
    r,g,b=[im[...,i].astype(int) for i in range(3)]
    olive=(g>b+35)&(r>b+30)&(abs(r-g)<30)
    brown=(r>g+25)&(g>b-5)&(r<200)
    white=(r>215)&(g>215)&(b>215)
    return olive,brown,white
out=[]
for f in range(1,57):
    im=np.asarray(Image.open(f"frames/f_{f:04d}.png").convert("RGB"))
    olive,brown,white=seg(im)
    ys,xs=np.nonzero(olive)
    d={"f":f,"olive_area":int(olive.sum()),"olive_bbox":[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())]}
    # head top: topmost olive pixel with x in body-centre band
    cx=np.median(xs); band=olive[:, int(cx)-40:int(cx)+40]
    d["head_top_y"]=int(np.nonzero(band.any(1))[0].min())
    # eyes: white blobs in upper half, x>cx-80
    wy,wx=np.nonzero(white[:300])
    sel=(wx>cx-90)&(wx<cx+90)
    d["eye_y"]=round(float(wy[sel].mean()),1) if sel.any() else None
    d["eye_x"]=round(float(wx[sel].mean()),1) if sel.any() else None
    d["eye_px"]=int(sel.sum())
    # left fist (screen right): rightmost olive extreme
    i=np.argmax(xs); d["fistL_extreme"]=[int(xs[i]),int(ys[i])]
    # right side / club: brown pixels in top 300 rows left of centre
    by,bx=np.nonzero(brown)
    c=(by<330)&(bx<cx-60)
    d["club_top"]=[int(bx[c][np.argmin(by[c])]),int(by[c].min())] if c.any() else None
    d["club_left_x"]=int(bx[c].min()) if c.any() else None
    d["club_centroid"]=[round(float(bx[c].mean()),1),round(float(by[c].mean()),1)] if c.any() else None
    # belt: brown in centre band between 280..460
    bb=(by>250)&(by<470)&(abs(bx-cx)<60)
    d["belt_y"]=round(float(by[bb].mean()),1) if bb.any() else None
    # feet: brown bottom rows
    fb=by>440
    d["feet_bbox"]=[int(bx[fb].min()),int(by[fb].min()),int(bx[fb].max()),int(by[fb].max())] if fb.any() else None
    d["body_cx"]=round(float(cx),1)
    out.append(d)
json.dump(out,open("measure.json","w"),indent=1)
for d in out:
    print(d["f"],d["olive_area"],d["olive_bbox"],"top",d["head_top_y"],"eye",d["eye_x"],d["eye_y"],d["eye_px"],"fL",d["fistL_extreme"],"club",d["club_top"],d["club_left_x"],"belt",d["belt_y"],"feet",d["feet_bbox"],"cx",d["body_cx"])
