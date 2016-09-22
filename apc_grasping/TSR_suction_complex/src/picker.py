import sys
import numpy as np
import random
import math
import tf
import transformations
from struct import unpack
from mayavi import mlab
from objloader import *

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def pick_callback(pick_obj):
	if pick_obj.actor in mesh.actor.actors:
		print pick_obj.mapper_position
	else:
		print ""

fname = ""
#Reading arguments
if len(sys.argv) > 1:
	fname = sys.argv[1]
else:
	sys.exit()

fnorms = []
normals = []
if fname.endswith('.stl'):
	f = pen(fname, 'rb')

	Header = f.read(80)
	nn = f.read(4)
	Numtri = unpack('i', nn)[0]
	record_dtype = np.dtype([('normals', np.float32,(3,)), ('Vertex1', np.float32,(3,)), ('Vertex2', np.float32,(3,)), ('Vertex3', np.float32,(3,)), ('atttr', '<i2',(1,))])
	data = numpy.fromfile(f, dtype = record_dtype, count = Numtri)
	f.close()

	Normals = data['normals']
	Vertex1 = data['Vertex1']
	Vertex2 = data['Vertex2']
	Vertex3 = data['Vertex3']

	x = []
	y = []
	z = []
	for i in range(len(Vertex1)):
		x.append(Vertex1[i][0])
		x.append(Vertex2[i][0])
		x.append(Vertex3[i][0])
		y.append(Vertex1[i][1])
		y.append(Vertex2[i][1])
		y.append(Vertex3[i][1])
		z.append(Vertex1[i][2])
		z.append(Vertex2[i][2])
		z.append(Vertex3[i][2])

	triangles = [(i, i+1, i+2) for i in range(0, len(x), 3)]

if fname.endswith('.obj'):
	obj = OBJ(sys.argv[1], swapyz=True)
	x = [v[0] for v in obj.vertices]
	y = [v[1] for v in obj.vertices]
	z = [v[2] for v in obj.vertices]
	triangles = [[f[0][0]-1, f[0][1]-1, f[0][2]-1] for f in obj.faces]
	fnorms = [f[1] for f in obj.faces]
	normals = obj.normals

mesh = mlab.points3d(x, y, z, color = (1,1,1), scale_factor=0.01)
trmesh = mlab.triangular_mesh(x, y, z, triangles, opacity=0.4)
fig = mlab.gcf()
picker = fig.on_mouse_pick(pick_callback)
picker.tolerance = 0.001
#mlab.show()

while True:
	cmd = raw_input('> ')
	cmds = cmd.split()
	if not cmds:
		continue
	for index, string in enumerate(cmds):
                cmds[index] = string.lower()
        if cmds[0] == 'quit':
		sys.exit()
	if cmds[0] == 'exit':
		sys.exit()
	if cmds[0] == 'sample':
		count = 100
		if True: #try:
			if len(cmds) > 1:
				count = int(cmds[1])
		
			output = open(fname[:-4] + ".tsrs", 'w')
			for i in range(count):
				cur = random.randint(0, len(x) - 1)
				output.write("- !BoxTSR\n  extents:\n")
				output.write("  - [%.6f, %.6f]\n" % (x[cur], x[cur]))
				output.write("  - [%.6f, %.6f]\n" % (y[cur], y[cur]))
				output.write("  - [%.6f, %.6f]\n" % (z[cur], z[cur]))
				for j in range(len(triangles)):
					t = triangles[j]
					id = -1
					if (t[0] == cur):
						id = 0
					if (t[1] == cur):
						id = 1
					if (t[2] == cur):
						id = 2
					if (id == -1):
						continue
					norm = normals[fnorms[j][id] - 1]
					nx = -norm[0]
					ny = -norm[1]
					nz = -norm[2]
					"""
					if math.fabs(ny) > 0.0001:
						rotz = math.atan(nx / ny)
					elif nx > 0:
						rotz = math.pi / 2
					else:
						rotz = - math.pi / 2
					if math.fabs(nz) > 0.0001:
						rotx = -math.atan(math.sqrt(nx*nx + ny*ny) / nz)
					else:
						rotx = -math.pi / 2
					# since calculated values are rotations from normal to manipulator
					rotz = rotz
					rotx = -rotx
					"""
					normal = [nx, ny, nz]
					#print "Normal: ",
					#print normal
					manip = [0, 0, 1]
					axis = numpy.cross(normal, manip)
					axis = axis / np.linalg.norm(axis)
					#print axis
					ax = axis[0]
					ay = axis[1]
					az = axis[2]
					angle = math.acos(numpy.dot(normal, manip) / np.linalg.norm(normal))
					s=math.sin(angle)
					c=math.cos(angle)
					t=1-c
					if (ax*ay*t + az*s) > 0.998:
						heading = 2*math.atan2(ax*math.sin(angle/2), math.cos(angle/2))
						attitude = math.pi/2
						bank = 0
					elif (ax*ay*t + az*s) < -0.998:
						heading = -2*math.atan2(ax*math.sin(angle/2), math.cos(angle/2))
						attitude = -math.pi/2;
						bank = 0
					else:
						heading = math.atan2(ay * s- ax * az * t , 1 - (ay*ay+ az*az ) * t)
						attitude = math.asin(ax * ay * t + az * s)
						bank = math.atan2(ax * s - ay * az * t , 1 - (ax*ax + az*az) * t)

					euler = [heading, attitude, bank]
					#print [e / math.pi * 180 for e in euler]
					rotx = bank
					rotz = attitude
					roty = heading
					# check if applied rotation results in proper normal direction
					normal = [[nx], [ny], [nz]]
					manip = [[0], [0], [1]]
					#A1 = [[math.cos(rotz), math.sin(rotz), 0], [-math.sin(rotz), math.cos(rotz), 0], [0, 0, 1]]
					#A2 = [[1, 0, 0], [0, math.cos(roty), math.sin(roty)], [0, -math.sin(roty), math.cos(roty)]]
					#A3 = [[math.cos(rotx), math.sin(rotx), 0], [-math.sin(rotx), math.cos(rotx), 0], [0, 0, 1]]
					#A3 = [[1, 0, 0], [0, math.cos(rotx), math.sin(rotx)], [0, -math.sin(rotx), math.cos(rotx)]]
					Ax = [[1, 0, 0], [0, math.cos(rotx), math.sin(rotx)], [0, -math.sin(rotx), math.cos(rotx)]]
					Ay = [[math.cos(roty), 0, -math.sin(roty)], [0, 1, 0], [math.sin(roty), 0, math.cos(roty)]]
					Az = [[math.cos(rotz), math.sin(rotz), 0], [-math.sin(rotz), math.cos(rotz), 0], [0, 0, 1]]
					rotation = np.dot(Ax, Az)
					rotation = np.dot(rotation, Ay)
					#rotation = np.dot(rotation, manip)
					na, nb, nc = transformations.euler_from_matrix(rotation, 'szyx')
					#na = 1.2
					#nb = math.pi / 4
					#nc = math.pi / 4
					matrix = transformations.euler_matrix(na, nb, nc, 'szyx')
					result = np.dot([line[:3] for line in matrix[:3]], manip)
					#print "Rotated manipulator: ",
					#print result
					#print normal
					#print rotation
					#na = angle_between(norm, (1, 0, 0))
					#nb = angle_between(norm, (0, 1, 0))
					#nc = angle_between(norm, (0, 0, 1))
					#na *= 180/math.pi
					#nb *= 180/math.pi
					#nc *= 180/math.pi
					output.write("  - [%.6f, %.6f]\n" % (na, na))
					output.write("  - [%.6f, %.6f]\n" % (nb, nb))
					output.write("  - [%.6f, %.6f]\n" % (nc, nc))
#mlab.plot3d([nx, nx + r * ], [g.vertices[path[v]].y, g.vertices[path[v+1]].y], [g.vertices[path[v]].z, g.vertices[path[v+1]].z], color = cfg.path_color, opacity = cfg.path_opacity, tube_radius = cfg.path_radius * zoom())
					break
			print "Done"
		else: #except:
			print "Failed :("
		output.close()
	else:
		try:
			num = float(cmds[0])
			mesh.glyph.glyph.scale_factor = num
		except:
			pass
