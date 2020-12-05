#!/usr/bin/env python3
#coding=utf-8
"""Convert cartesian coordinate system to geodetic system.

usage: xyz2blh.py [-h] [-v] -x <num> -y <num> -z <num>

optional arguments:
  -h, --help     show this help message and exit
  -v, --version  show program's version number and exit
  -x <val>       the X coordinate
  -y <val>       the Y coordinate
  -z <val>       the Z coordinate

"""
from __future__ import division, print_function
import argparse
import math

A = 6378137.0
B = 6356752.314245


def xyz2blh(x, y, z):
    """Convert XYZ coordinates to BLH,
    return tuple(latitude, longitude, height).

    Example 1, convert BJFS(Beijing, China) XYZ to BLH:

    BLH position calculated by TEQC software:
    - latitute: 39.6086
    - longitude: 115.8928
    - height: 112.78
    >>> lat, lon, hgt = xyz2blh(-2148778.283, 4426643.490, 4044675.194)
    >>> round(lat, 4), round(lon, 4), round(hgt, 2)
    (39.6086, 115.8928, 112.78)

    Example 2, convert BOGT(Bogota, Colombia) XYZ to BLH:

    BLH position calculated by TEQC software:
    - latitute: 4.6401
    - longitude: -74.0806
    - height: 2585.69
    >>> lat, lon, hgt = xyz2blh(1744433.521, -6116034.660, 512736.584)
    >>> round(lat, 4), round(lon, 4), round(hgt, 2)
    (4.6401, -74.0806, 2585.69)
    """
    e = math.sqrt(1 - (B**2)/(A**2))
    # calculate longitude, in radians
    longitude = math.atan2(y, x)

    # calculate latitude, in radians
    xy_hypot = math.hypot(x, y)

    lat0 = 0
    latitude = math.atan(z / xy_hypot)

    while abs(latitude - lat0) > 1E-9:
        lat0 = latitude
        N = A / math.sqrt(1 - e**2 * math.sin(lat0)**2)
        latitude = math.atan((z + e**2 * N * math.sin(lat0)) / xy_hypot)
    # calculate height, in meters
    height = z / math.sin(latitude) - N * (1 - e**2)
    # convert angle unit to degrees
    longitude = math.degrees(longitude)
    latitude = math.degrees(latitude)

    return latitude, longitude, height


def main():
    """Main function."""
    args = init_args()
    latitude, longitude, height = xyz2blh(args.x, args.y, args.z)
    message = 'latitude: {:.5f}, longitude: {:.5f}, height: {:.3f}'
    print(message.format(latitude, longitude, height))

    return 0


def init_args():
    """Initialize user input"""
    # Create a arguments parser
    parser = argparse.ArgumentParser(
        description="Convert cartesian coordinate system to geodetic system.")
    # Add arguments
    parser.add_argument('-v', '--version', action='version',
                        version='%(prog)s 0.0.1')
    parser.add_argument('-x', metavar='<val>', dest='x', type=float,
                        required=True, help='the X coordinate')
    parser.add_argument('-y', metavar='<val>', dest='y', type=float,
                        required=True, help='the Y coordinate')
    parser.add_argument('-z', metavar='<val>', dest='z', type=float,
                        required=True, help='the Z coordinate')

    # Parse arguments
    return parser.parse_args()


if __name__ == '__main__':
    main()
