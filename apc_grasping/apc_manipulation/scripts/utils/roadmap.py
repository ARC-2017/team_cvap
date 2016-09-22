#!/usr/bin/env python
""" This file contains classes for Roadmap path planning (for the specific case of the APC with Baxter)"""

import yaml
import math
import Queue
import operator


def computeTrajectoryLength(traj):
    """ Computes the length of a MoveIt trajectory"""
    accDist = 0.0
    for i in range(1, len(traj.joint_trajectory.points)):
        jtpIM = traj.joint_trajectory.points[i - 1]
        jtpI = traj.joint_trajectory.points[i]
        dist = 0.0
        for j in range(len(jtpI.positions)):
            dist += (jtpI.positions[j] - jtpIM.positions[j]) * (jtpI.positions[j] - jtpIM.positions[j])
        accDist += math.sqrt(dist)
    return accDist


def computeDistanceConfigs(configA, configB):
    """ Computes the distance between two configurations. Both configurations are expected
        to be dictionaries, where the keys are the joint names mapping to the respective joint value. """
    dist = 0.0
    for joint in configA.iterkeys():
        dist += (configA[joint] - configB[joint]) * (configA[joint] - configB[joint])
    return math.sqrt(dist)


class HeapEntry:
    def __init__(self, nodeId, parentId, gscore):
        self.nodeId = nodeId
        self.parentId = parentId
        self.gscore = gscore


class RoadmapEdge(yaml.YAMLObject):
    """ A unidirectional edge in the roadmap. An edge connects two nodes through a trajectory. """
    yaml_tag = u'!RoadmapEdge'

    def __init__(self, fromNode, toNode, traj):
        self._fromNode = fromNode
        self._toNode = toNode
        self._traj = traj
        self._trajLength = computeTrajectoryLength(traj)

    def __repr__(self):
        return "%s(fromNode=%r, toNode=%r, traj=%r, trajLength=%f)" % (
            self.__class__.__name__, self._fromNode, self._toNode, self._traj, self._trajLength)


class RoadmapNode(yaml.YAMLObject):
    """ A node in the roadmap. It represents a named configuration.
        Once this instance is added to a roadmap, do not add this instance to any other roadmap!
    """
    yaml_tag = u'!RoadmapNode'

    def __init__(self, config, name):
        self._config = config
        self._name = name
        self._edges = []
        self._id = -1

    def addEdge(self, toNode, traj):
        """ Add an edge from this node to the given node using the given trajectory. """
        edge = RoadmapEdge(self, toNode, traj)
        self._edges.append(edge)

    def removeEdge(self, toNode):
        remainingEdges = []
        for edge in self._edges:
            if edge._toNode == toNode:
                continue
            else:
                remainingEdges.append(edge)
        self._edges = remainingEdges

    def getConfig(self):
        return self._config

    def getName(self):
        return self._name

    def getId(self):
        return self._id

    def getDegree(self):
        return len(self._edges)

    def isAdjacent(self, toNode):
        for edge in self._edges:
            if edge._toNode == toNode:
                return True
        return False

    def __repr__(self):
        return "%s(config=%r, name=%r, edges=%r)" % (
            self.__class__.__name__, self._config, self._name, len(self._edges))


class Roadmap(yaml.YAMLObject):
    """ The roadmap class. """
    yaml_tag = u'!Roadmap'

    def __init__(self):
        self._nodes = []

    def addNode(self, node):
        """ Adds a node to the roadmap. The edges are stored within each node.
            Note that a node can only be member of one roadmap at a time!
        """
        node._id = len(self._nodes)
        self._nodes.append(node)

    def __repr__(self):
        return "%s(nodes=%r)" % (self.__class__.__name__, self._nodes)

    def getNode(self, idx):
        if idx in range(len(self._nodes)):
            return self._nodes[idx]
        else:
            return None

    def getTotalNumEdges(self):
        numEdges = 0
        for node in self._nodes:
            numEdges += node.getDegree()
        return numEdges

    def removeEdge(self, nodeAName, nodeBName):
        nodeA = self.getNodeFromName(nodeAName)
        nodeB = self.getNodeFromName(nodeBName)
        if nodeA is None or nodeB is None:
            return False
        nodeA.removeEdge(nodeB)
        nodeB.removeEdge(nodeA)

    def isAdjacent(self, nodeAName, nodeBName):
        nodeA = self.getNodeFromName(nodeAName)
        nodeB = self.getNodeFromName(nodeBName)
        if nodeA is None or nodeB is None:
            return False
        return nodeA.isAdjacent(nodeB) and nodeB.isAdjacent(nodeA)

    def getNodeFromName(self, name):
        for node in self._nodes:
            if node.getName() == name:
                return node
        return None

    def getNodes(self):
        return self._nodes

    def getNearestNeighbors(self, config, nNeighbors=1):
        """ Returns the nNeighbors nearest neighbors for the given configuration """
        # linear search does the job here since we probably have less than 200 nodes
        pq = Queue.PriorityQueue()
        # minDist = float('inf')
        # nearestNeighbor = None
        for node in self._nodes:
            dist = computeDistanceConfigs(config, node.getConfig())
            pq.put((dist, node))
            # if dist < minDist:
            #     minDist = dist
            #     nearestNeighbor = node
        result = []
        for i in range(min(nNeighbors, len(self._nodes))):
            (dist, node) = pq.get()
            result.append(node)
        return result

    def getNewNodeName(self):
        return 'node_' + str(len(self._nodes))

    def planPath(self, startNode, goalNode):
        if startNode not in self._nodes:
            raise ValueError('The start node is not in this roadmap')
        if goalNode not in self._nodes:
            raise ValueError('The goal node is not in this roadmap')

        # Initialization
        pq = Queue.PriorityQueue()
        parents = [-1] * len(self._nodes)
        cheapestPath = [float('inf')] * len(self._nodes)
        # Add the start node
        he = HeapEntry(nodeId=startNode._id, parentId=startNode._id, gscore=0.0)
        currentF = computeDistanceConfigs(startNode._config, goalNode._config)
        pq.put((currentF, he))
        parents[startNode._id] = startNode._id
        cheapestPath[startNode._id] = 0.0
        # start the search
        while not pq.empty():
            (currentF, currentHeapEntry) = pq.get()
            # We may have a node in the pq multiple times, check whether this is the shortest path
            if currentHeapEntry.gscore > cheapestPath[currentHeapEntry.nodeId]:
                continue
            cheapestPath[currentHeapEntry.nodeId] = currentHeapEntry.gscore
            parents[currentHeapEntry.nodeId] = currentHeapEntry.parentId

            # have we reached our goal?
            if currentHeapEntry.nodeId == goalNode._id:
                return self.extractPath(parents, goalNode._id)

            # else expand this node
            currentNode = self._nodes[currentHeapEntry.nodeId]
            for edge in currentNode._edges:
                neighborId = edge._toNode._id
                neighborG = currentHeapEntry.gscore + edge._trajLength
                neighborF = neighborG + computeDistanceConfigs(edge._toNode._config, goalNode._config)
                if neighborG < cheapestPath[neighborId]:
                    he = HeapEntry(nodeId=neighborId, parentId=currentHeapEntry.nodeId, gscore=neighborG)
                    pq.put((neighborF, he))

        print 'No path found:('
        return None

    def extractPath(self, parents, goalNodeId):
        trajectories = []
        currentId = goalNodeId
        parentId = parents[goalNodeId]
        while currentId != parentId:
            parentNode = self._nodes[parentId]
            for edge in parentNode._edges:
                if edge._toNode._id == currentId:
                    trajectories.append(edge._traj)
                    break
            currentId = parentId
            parentId = parents[parentId]
        trajectories.reverse()
        return trajectories

    def isStronglyConnected(self):
        """ Checks whether each node is reachable from every other node
            (assuming that the graph is bidirectional).
            @return True, iff strongly connected (assuming bidirectionality holds)
        """
        nodeVisited = len(self._nodes) * [False]
        unexploredNodes = [self._nodes[0]]
        while len(unexploredNodes) > 0:
            currentNode = unexploredNodes.pop()
            if nodeVisited[currentNode._id]:
                continue
            for edge in currentNode._edges:
                if not nodeVisited[edge._toNode._id]:
                    unexploredNodes.append(edge._toNode)
            nodeVisited[currentNode._id] = True

        return reduce(operator.and_, nodeVisited, True)


def showTestTraj(traj):
    for edge in traj:
        print edge[0]

if __name__ == '__main__':
    roadmap = Roadmap()
    nodeA = RoadmapNode({'x': 0.0, 'y': 0.0}, 'nodeA')
    nodeB = RoadmapNode({'x': 0.1, 'y': 0.1}, 'nodeB')
    nodeC = RoadmapNode({'x': 0.1, 'y': 0.0}, 'nodeC')
    nodeD = RoadmapNode({'x': 0.2, 'y': 0.0}, 'nodeD')
    nodeE = RoadmapNode({'x': 0.2, 'y': 0.1}, 'nodeE')
    nodeF = RoadmapNode({'x': 0.3, 'y': 0.0}, 'nodeF')
    nodeG = RoadmapNode({'x': 0.3, 'y': 0.01}, 'nodeG')

    nodeA.addEdge(nodeC, ('A->C', 0.1))
    nodeA.addEdge(nodeF, ('A->F', 0.4))
    nodeB.addEdge(nodeA, ('B->A', 0.2))
    nodeB.addEdge(nodeC, ('B->C', 0.1))
    nodeB.addEdge(nodeD, ('B->D', 0.2))
    nodeB.addEdge(nodeE, ('B->E', 0.1))
    nodeC.addEdge(nodeB, ('C->B', 0.1))
    nodeC.addEdge(nodeF, ('C->F', 0.2))
    nodeD.addEdge(nodeC, ('D->C', 0.12))
    nodeE.addEdge(nodeD, ('E->D', 0.11))
    nodeE.addEdge(nodeG, ('E->G', 0.3))
    nodeF.addEdge(nodeC, ('F->C', 0.2))
    nodeF.addEdge(nodeG, ('F->G', 0.02))

    roadmap.addNode(nodeA)
    roadmap.addNode(nodeB)
    roadmap.addNode(nodeC)
    roadmap.addNode(nodeD)
    roadmap.addNode(nodeE)
    roadmap.addNode(nodeF)
    roadmap.addNode(nodeG)

    import IPython
    IPython.embed()
