#!/usr/bin/env python
import openravepy as orpy
import IPython
import argparse

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='This script computes an optimal shelf pose in respect to Baxter.')
  parser.add_argument('--viewer', dest='viewer', action='store_true', 
                      default=False, help='Set if you want a viewer to show up.')
  args = parser.parse_args()
  env = orpy.Environment()
  env.Load('/home/joshua/projects/catkin_ws/src/baxter_custom_ikfast/baxter_arm.right.dae')
  if args.viewer:
    env.SetViewer('qtcoin')
  IPython.embed()
