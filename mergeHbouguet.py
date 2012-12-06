#!/usr/bin/env python
"""
First file should contain parameters from Bouguet's toolbox, as
required by trackem_ros, (e.g., as extracted by bouguet2npz.py), and
the second should contain a projective transformation matrix H (e.g.,
as obtained with homcomp.py).

If no output filename is given, then automatically make it of the form
camcalibration-YYYYMMDD-HHMMSS.npz, which uses the current date.


SCL; 6 Dec 2012.
"""

import sys
import time
import numpy as np


if len(sys.argv) != 3 and len(sys.argv) != 4:
    print "Usage: mergeHbouguet.py FILE1 FILE2 [OUTPUT]"
    exit(1)

if len(sys.argv) == 3:
    outname = "camcalibration-"+time.strftime("%Y%m%d-%H%M%S")+".npz"
else:
    outname = sys.argv[3]

intrin_param = np.load(sys.argv[1])
extrin_param = np.load(sys.argv[2])

np.savez(outname,
         fc=intrin_param["fc"], cc=intrin_param["cc"], kc=intrin_param["kc"],
         H=extrin_param["H"])
