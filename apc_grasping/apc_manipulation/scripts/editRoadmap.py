import utils.roadmap
import pickle
import IPython
import argparse


def saveRoadmap(filename, roadmap):
    afile = open(filename, 'w')
    pickle.dump(roadmap, afile)
    afile.close()

roadmapPath = '/home/joshua/projects/catkin_ws/src/apc_grasping/apc_manipulation/data/modified_roadmap.pickle'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Interactive shell that allows modifiying a roadmap!')
    parser.add_argument('roadmapPath',type=str,
                        help='The path to the roadmap to load.')
    args = parser.parse_args()
    afile = open(args.roadmapPath, 'r')  
    roadmap = pickle.load(afile)
    afile.close()
    IPython.embed()