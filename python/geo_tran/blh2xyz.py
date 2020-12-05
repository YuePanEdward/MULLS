#!/usr/bin/env python3
#coding=utf-8
"""Convert geodetic coordinate system to cartesian system.

usage: blh2xyz.py [-h] [-v] -lat lat -lon lon -hgt height

Convert BLH coordinates to XYZ.

optional arguments:
  -h, --help     show this help message and exit
  -v, --version  show program's version number and exit
  -lat lat       the lattitude
  -lon lon       the longitude
  -hgt height    the height

"""
from __future__ import division, print_function
import argparse
import math

A = 6378137.0
B = 6356752.314245


def blh2xyz(latitude, longitude, height):
    """Convert BLH coordinates to XYZ.
    return tuple(X, Y, Z).

    Example 1, convert BJFS(Beijing, China) BLH to XYZ:

    XYZ position calculated by TEQC software:
    - X: -2148748
    - Y: 4426656
    - Z: 4044670

    >>> x, y, z = blh2xyz(39.608611, 115.892456, 108.0420)
    >>> round(x), round(y), round(z)
    (-2148748, 4426656, 4044670)

    Example 2, convert BOGT(Bogota, Colombia) BLH to XYZ:

    XYZ position calculated by TEQC software:
    - X: 1744394
    - Y: -6116025
    - Z: 512728

    >>> x, y, z = blh2xyz(4.640045, -74.080950, 2563.1791)
    >>> round(x), round(y), round(z)
    (1744394, -6116025, 512728)
    """
    # convert angle unit to radians
    latitude = math.radians(latitude)
    longitude = math.radians(longitude)

    e = math.sqrt(1 - (B**2)/(A**2))
    N = A / math.sqrt(1 - e**2 * math.sin(latitude)**2)
    # calculate X, Y, Z
    X = (N + height) * math.cos(latitude) * math.cos(longitude)
    Y = (N + height) * math.cos(latitude) * math.sin(longitude)
    Z = (N * (1 - e**2) + height) * math.sin(latitude)

    return X, Y, Z


def main():
    """Main function."""
    args = init_args()
    x_val, y_val, z_val = blh2xyz(args.b, args.l, args.h)
    message = 'X: {:.5f}, Y: {:.5f}, Z: {:.5f}'
    print(message.format(x_val, y_val, z_val))

    return 0


def init_args():
    """Initialize user input."""
    # create a arguments parser
    parser = argparse.ArgumentParser(
        description="Convert geodetic coordinate system to cartesian system.")
    # add arguments
    parser.add_argument('-v', '--version', action='version',
                        version='%(prog)s 0.0.1')
    parser.add_argument('-lat', metavar='lat', dest='b', type=float,
                        required=True, help='the lattitude')
    parser.add_argument('-lon', metavar='lon', dest='l', type=float,
                        required=True, help='the longitude')
    parser.add_argument('-hgt', metavar='height', dest='h', type=float,
                        required=True, help='the height')

    # parse arguments
    return parser.parse_args()


if __name__ == '__main__':
    main()
