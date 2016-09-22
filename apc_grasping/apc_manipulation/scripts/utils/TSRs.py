#!/usr/bin/env python
import yaml
import random

# def numpyArray_representer(dumper, data):
#     """ Produces a yaml node representing a numpy array """
#     return dumper.represent_scalar(u'!ndarray', u'%s' % data.tolist())
#
#
# def numpyArray_constructor(loader, node):
#     """ Creates a numpy array from its yaml description """
#     value = loader.construct_scalar(node)
#     return numpy.array(map(float, value.replace(']', '').replace('[', '').split(',')))
#
#
# def addNumpyYaml():
#     yaml.add_representer(numpy.ndarray, numpyArray_representer)
#     yaml.add_constructor(u'!ndarray', numpyArray_constructor)


class BoxTSR(yaml.YAMLObject):
    yaml_tag = u'!BoxTSR'
    discrete_tsr_area = (5e-3*5e-3)

    def __init__(self, extents=None):
        if extents is None:
            self.extents = []
            for r in range(6):
                self.extents.append([0.0, 0.0])
        else:
            self.extents = extents

    def __repr__(self):
        return "%s(extents=%r)" % (self.__class__.__name__, self.extents)

    def getVolume(self):
        edges = map(lambda x: x[1] - x[0], self.extents[:3])
        return reduce(lambda x, y: x * y, edges)

    def getArea(self):
        edges = map(lambda x: x[1] - x[0], self.extents[:3])

        nonzero_edges = filter(lambda x: abs(x)>1e-4, edges)

        if len(nonzero_edges) > 1:
            area = reduce(lambda x, y: x*y, nonzero_edges)

        else:
            area = self.discrete_tsr_area

        return area

    def isDiscrete(self):
        return self.getArea()<=(self.discrete_tsr_area)

    def drawSample(self, posPadding, rotPadding):
        """ Returns a sample.
            @param posPadding - minimal distance to position boundaries
            @param rotPadding - minimal distance to rotation boundaries
            @return (pose, posPadding, rotPadding) - pose as list [x, y, z, r, p, y]
            posPadding is updated posPadding if it was too large, rotPadding is
            updated rotPadding if it was too large """
        sample = 6 * [0.0]
        minPosPadding, minRotPadding = posPadding, rotPadding
        for i in range(6):
            padding = posPadding if i < 3 else rotPadding
            intervalLength = abs(self.extents[i][1] - self.extents[i][0])
            if 2 * padding >= intervalLength:
                sample[i] = self.extents[i][0] + intervalLength / 2.0
                if i < 3:
                    minPosPadding = min(intervalLength / 2.0, minPosPadding)
                else:
                    minRotPadding = min(intervalLength / 2.0, minRotPadding)
            else:
                sample[i] = random.triangular(self.extents[i][0] + padding,
                                              self.extents[i][1] - padding,
                                              0.5 * (self.extents[i][0] + self.extents[i][1]))
                # sample[i] = random.uniform(self.extents[i][0] + padding,
                #                            self.extents[i][1] - padding)
        return sample, minPosPadding, minRotPadding

        # def toYaml(self):
        #     tsrAsMap = {}
        #     tsrAsMap['type'] = 'BoxTSR'
        #     dimNames = ['x', 'y', 'z', 'r', 'p', 'y']
        #     for i in range(len(dimNames)):
        #         tsrAsMap[dimNames[i]+ '_min'] = self.extents[i, 0]
        #         tsrAsMap[dimNames[i]+ '_max'] = self.extents[i, 1]
        #     return yaml.dump(tsrAsMap)
        #
        # def fromYaml(self, yamlString):
        #     tsrAsMap = yaml.load(yamlString)
        #     if not tsrAsMap['type'] == 'BoxTSR':
        #         raise ValueError('Could not parse yaml string:\n ' + yamlString '\n Invalid type!')
        #     dimNames = ['x', 'y', 'z', 'r', 'p', 'y']
        #     for i in range(len(dimNames)):
        #         self.extents[i, 0] = tsrAsMap[dimNames[i + '_min']]
        #         self.extents[i, 1] = tsrAsMap[dimNames[i + '_max']]
