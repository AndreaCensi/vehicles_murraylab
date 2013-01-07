#!/usr/bin/env python
"""
Find physical coordinates of a given point using a calibration as
recognized by trackem_ros.  Invocation is of the form

  $ ./overhead-interactive.py animal.png house.npz [FILE]

where animal.png is an image from the overhead camera and house.npz is
a calibration file suitable for use by the forwardcal.py node of
trackem_ros.  FILE is the name of a plaintext file to which the list
of points clicked should be saved in a format that can be read by the
function numpy.loadtxt().  If no FILE is specified, then no such list
is saved to disk.


SCL; 7 Jan 2013.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def normalize(x_p, f=None, pp=None, kc=None, num_it=5):
    """Normalize image coordinate.

    Equation (19) (in Section 3) of

      J. Heikkil\"{a} and O. Silv\'{e}n (1997).  A Four-step Camera
      Calibration Procedure with Implicit Image Correction.  Proceedings
      of Computer Vision and Pattern Recognition ({CVPR}).

    Also see the parameters description page of J.-Y. Bouguet's
    "Camera Calibration Toolbox for Matlab" website;
    http://vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html

    f is focal length; pp is principal point; kc is vector of radial
    and tangential distortion parameters.

    ...copied verbatim from trackem_ros, 2012-01-05 15:55
    """
    if f is None:
        f = np.ones(2)
    if pp is None:
        pp = np.zeros(2)
    if kc is None:
        kc = np.zeros(5)

    x_d = np.array([(x_p[0]-pp[0])/f[0],
                    (x_p[1]-pp[1])/f[1]])

    x_n = x_d.copy()
    for i in range(num_it):
        r2 = np.sum(x_n**2)
        dx = np.array([2*kc[2]*x_n[0]*x_n[1] + kc[3]*(r2 + 2*x_n[0]**2),
                       kc[2]*(r2 + 2*x_n[1]**2) + 2*kc[3]*x_n[0]*x_n[1]])
        x_n = (x_d-dx)/(1 + kc[0]*r2 + kc[1]*r2**2 + kc[4]*r2**3)

    return x_n


def pixel2m(x_p, f, pp, kc, H):
    """Convert raw image coordinate to actual physical position.

    ...copied verbatim from trackem_ros, 2012-01-05 15:55
    """
    x_n = np.array([0., 0, 1])
    x_n[:2] = (normalize(np.asarray(x_p), f=f, pp=pp, kc=kc)*f+pp)[:,0]
    return np.dot(H, x_n)


############################################################
## Execution begins here
if len(sys.argv) < 3:
    print "Usage: %s FILE.png FILE.npz [FILE.txt]" % sys.argv[0]
    exit(1)

I = mpimg.imread(sys.argv[1])

cal_data = np.load(sys.argv[2])
fc = cal_data["fc"]
cc = cal_data["cc"]
kc = cal_data["kc"]
H = cal_data["H"]

points_clicked = []


def onclick(event):
    if event.xdata is None or event.ydata is None:
        return
    points_clicked.append(pixel2m((event.xdata, event.ydata),
                                  f=fc, pp=cc, kc=kc, H=H)[:2])
    print points_clicked[-1]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.imshow(I)
cid = fig.canvas.mpl_connect("button_press_event", onclick)

plt.show()

# Save list to disk if requested
if len(sys.argv) >= 4:
    print "Saving points that were clicked to "+sys.argv[3]+"..."
    with open(sys.argv[3], "w") as f:
        f.write("\n".join([str(x[0])+" "+str(x[1]) for x in points_clicked])+"\n")
