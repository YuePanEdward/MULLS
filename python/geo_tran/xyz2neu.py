#!/usr/bin/env python3
#coding=utf-8
"""Convert cartesian coordinate system to site-center system.

"""
from __future__ import division, print_function
import argparse
import math

from xyz2blh import xyz2blh

A = 6378137.0
E2 = 0.00669438
E4 = 0.0067394968
B = 0.1


def xyz2neu(x0, y0, z0, x, y, z):
    """Convert cartesian coordinate system to site-center system.

    Input paraments:
    - x0, y0, z0: coordinate of centra site,
    - x, y, z: coordinate to be converted.

    Example: Use coordinate of BJFS IGS site
    >>> north, east, up = xyz2neu(-2148747.998, 4426652.444, 4044675.151,
    ... -2148745.727, 4426649.545, 4044668.469)
    >>> round(north, 2), round(east, 2), round(up, 2)
    (-2.85, -0.78, -7.03)
    """
    # calculate the lat, lon and height of center site
    lat, lon, _ = xyz2blh(x0, y0, z0)
    # convert angle unit to radians
    lat, lon = math.radians(lat), math.radians(lon)
    # calculate NEU
    north = (-math.sin(lat) * math.cos(lon) * (x - x0) -
             math.sin(lat) * math.sin(lon) * (y - y0) +
             math.cos(lat) * (z - z0))
    east = -math.sin(lon) * (x - x0) + math.cos(lon) * (y - y0)
    up = (math.cos(lat) * math.cos(lon) * (x- x0) +
          math.cos(lat) * math.sin(lon) * (y - y0) +
          math.sin(lat) * (z - z0))

    return north, east, up


def main():
    """Main function."""
    args = init_args()
    north, east, up = xyz2neu(args.x0, args.y0, args.z0, args.x, args.y, args.z)
    message = ('Center: {0:.2f}, {1:.2f}, {2:.2f}\n'
               'North: {3:.2f}, East: {4:.2f}, Up: {5:.2f}')
    print(message.format(args.x0, args.y0, args.z0, north, east, up))

    return 0


def init_args():
    """Initialze user input."""
    parser = argparse.ArgumentParser(
        description='Convert cartesian coordinate system to site-center NEU.')
    # add paraments
    parser.add_argument('-x0', metavar='<x0>', dest='x0', type=float,
                        help='topocentric X coordinate.')
    parser.add_argument('-y0', metavar='<y0>', dest='y0', type=float,
                        help='topocentric Y coordinate.')
    parser.add_argument('-z0', metavar='<z0>', dest='z0', type=float,
                        help='topocentricZ coordinate.')
    parser.add_argument('-x', metavar='<x>', dest='x', type=float,
                        help='X coordinate will convert.')
    parser.add_argument('-y', metavar='<y>', dest='y', type=float,
                        help='Y coordinate will convert.')
    parser.add_argument('-z', metavar='<z>', dest='z', type=float,
                        help='Z coordinate will convert.')

    return parser.parse_args()


if __name__ == '__main__':
    main()
