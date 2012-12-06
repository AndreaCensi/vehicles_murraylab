#!/usr/bin/env python
"""
DLT for 2-dimensional homography, following treatment by R. Hartley
and A. Zisserman in "Multiple View Geometry in computer vision",
section 4.1 of the second edition, pages 88--93.


SCL; 6 Dec 2012.
"""

import sys
import numpy as np
import numpy.linalg as la


if __name__ == "__main__":
    if len(sys.argv) == 1:
        outname = "H.npz"
    elif (len(sys.argv) == 2) and (sys.argv[1] != "-h") and (sys.argv[1] != "--help"):
        outname = sys.argv[1]
    else:
        print "Usage: homcomp.py [OUTPUT.npz]"
        exit(1)

    side = 3.048  # meters

    x = []  # in homogeneous coordinates
    x.append(np.array([154.11615433,372.16134102,1]))  # origin in physical space
    x.append(np.array([446.94656805,363.56039672,1]))  # +x corner ("lower right" in physical space)
    x.append(np.array([450.1413595,79.98262313,1]))  # +x,+y corner ("upper right")
    x.append(np.array([149.93661171,66.99115358,1]))  # +y corner

    ref_x = []  # in homogeneous coordinates
    ref_x.append(np.array([0,0.,1]))
    ref_x.append(np.array([side,0,1]))
    ref_x.append(np.array([side, side,1]))
    ref_x.append(np.array([0, side,1]))

    # DLT
    A = []
    for i in range(len(x)):
        A.append(np.array([[0,0,0,
                            -x[i][0], -x[i][1], -1,
                            ref_x[i][1]*x[i][0], ref_x[i][1]*x[i][1], ref_x[i][1]*1],
                           [x[i][0], x[i][1], 1,
                            0,0,0,
                            -ref_x[i][0]*x[i][0], -ref_x[i][0]*x[i][1], -ref_x[i][0]*1]]))
    A = np.vstack(A)
    U, s, Vh = la.svd(A)
    H = Vh.T[:,-1]
    H = H.reshape((3,3))

    # Compute reasonable scaling
    scale = 0.
    for i in range(len(x)):
        x_mapped = np.dot(H, x[i])
        scale += 1/x_mapped[-1]
    scale /= len(x)
    H *= scale

    print H

    # Percent error
    for i in range(len(x)):
        print la.norm(ref_x[i] - np.dot(H, x[i]))/la.norm(ref_x[i])

    # Save the result
    np.savez(outname, H=H)
