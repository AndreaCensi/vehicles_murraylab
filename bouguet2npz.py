#!/usr/bin/env python
"""
Extract several calibration parameters obtained using J.-Y. Bouguet's
Camera Calibration Toolbox for Matlab
(http://vision.caltech.edu/bouguetj/calib_doc/)
and save them into a NumPy NPZ file.

SCL; 13 Nov 2012.
"""

import sys
import numpy as np
from scipy.io import loadmat


if __name__ == "__main__":
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        print "Usage: bouguet2npz.py INPUT.mat [OUTPUT.npz]"
        exit(1)

    matdata = loadmat(sys.argv[1])
    fc = matdata["fc"]  # focal length
    cc = matdata["cc"]  # principal point
    kc = matdata["kc"]  # radial and tangential distortion

    if len(sys.argv) == 2:
        outname = sys.argv[1]+".npz"
    else:
        outname = sys.argv[2]
    np.savez(outname, fc=fc, cc=cc, kc=kc)
