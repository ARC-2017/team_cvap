#!/usr/bin/env python

#   calib_rpy
#
#   Created on: May 25, 2016
#   Authors:   Francisco Vina
#             fevb <at> kth.se
#

#  Copyright (c) 2016, Francisco Vina, CVAP, KTH
#    All rights reserved.

#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#       * Redistributions of source code must retain the above copyright
#         notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#       * Neither the name of KTH nor the
#         names of its contributors may be used to endorse or promote products
#         derived from this software without specific prior written permission.

#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
#    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.





import sys
import PyKDL as kdl
import argparse
import numpy as np

# show the calib file in [X, Y, Z, Roll, Pitch, Yaw] format instead of horrendous quaternions
if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Show calibration file with roll pitch yaw angles')

    parser.add_argument('filename', metavar='filename', type=str, nargs='+',
                        help='Calibration file to read')

    args = parser.parse_args()
    filename = args.filename[0]

    array = np.loadtxt(filename)

    x = array[0]
    y = array[1]
    z = array[2]

    q = array[3:7]

    R = kdl.Rotation.Quaternion(*q)

    rpy = R.GetRPY()

    pose = np.array((x, y, z) + rpy)

    print('[x, y, z, roll, pitch, yaw]: ')
    print(np.array_str(pose, precision=4))




