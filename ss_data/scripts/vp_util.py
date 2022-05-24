import random
import numpy as np
import math
from map import *
from util import *
import sys
import os
import argparse


def getParameters():
    parser = argparse.ArgumentParser()
    parser.add_argument('--load', type=str, default=None, help="path to the viewpoints")
    parser.add_argument('--alter',type=bool, default=False)
    parser.add_argument('--mirror', type=bool, default=False)
    return parser.parse_args()


if __name__ == "__main__":
    args = getParameters()

    viewpoints = []
    for file in os.listdir(args.load):
        vpfile = os.path.join(args.load, file)
        vpGenerator = ViewPointGenerator()
        vps = vpGenerator.load(vpfile)
        for vp in vps:
            viewpoints.append(vp)

    if args.alter:
        viewpoints = alterTour(viewpoints)

    if args.mirror:
        viewpoints = mirrorTour(viewpoints)

    vpGenerator.save(os.path.join(args.load,"merged.txt"), viewpoints)
