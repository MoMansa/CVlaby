#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Quaternion 
from std_msgs.msg import Header
from tkinter import Tk, Canvas
from stl import mesh
import numpy as np
import random
 
class MazeFactoryROS:
    def __init__(self):
        rospy.init_node('maze_factory')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
 
    def insert_maze_model(self, maze_stl_filename):
        model_state = ModelState()
        model_state.model_name = "maze"
        model_state.pose = Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))
        model_state.reference_frame = "world"
        model_state.pose.position.z = -0.1  # Assurez-vous que le modèle de labyrinthe n'est pas enfoncé dans le sol
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        model_state.reference_frame = "world"
 
        with open(maze_stl_filename, 'rb') as f:
            model_state.model_mesh = f.read()
 
        self.set_model_state(model_state)
 
def create_maze(width, height):
    def create_maze(width, height):
    maze = [['#'] * width for _ in range(height)] # Rempli la matrice de # qui représente les murs
    
    def is_valid(x, y):
        return x >= 0 and y >= 0 and x < width and y < height
        
    def dig(x, y):
        maze[y][x] = ' ' # Initialise le labyrinthe (tableau)

        directions = [(2, 0), (-2, 0), (0, 2), (0, -2)] # Représente toutes les directions possibles
        random.shuffle(directions)  # Mélange le tableau pour plus de hasard

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(nx, ny) and maze[ny][nx] == '#': # Vérifie si se déplacer vers les 4 directions reste dans le labyrinthe et si ces nouvelles cases sont des murs
                maze[y + dy // 2][x + dx // 2] = ' ' # Creuse
                dig(nx, ny) # se réappelle

    start_x, start_y = random.randint(0, width // 2) * 2, random.randint(0, height // 2) * 2
    dig(start_x, start_y)

    maze[height - 1][width - 1] = ' ' # Sortie du labyrinthe

    return maze
 
def generate_stl_from_maze(maze, output_filename='maze.stl'):
    verts = []
    faces = []

    for y in range(len(maze)):
        for x in range(len(maze[y])):
            if maze[y][x] == ' ':
                verts.append([x, y, 0])
                verts.append([x + 1, y, 0])
                verts.append([x + 1, y + 1, 0])
                verts.append([x, y + 1, 0])

                face = [
                    [len(verts) - 4, len(verts) - 3, len(verts) - 2],
                    [len(verts) - 4, len(verts) - 2, len(verts) - 1]
                ]

                faces.extend(face)

    maze_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            maze_mesh.vectors[i][j] = verts[face[j]]

    maze_mesh.save(output_filename)
 
def display_maze_with_tkinter(maze):
    root = Tk()
    root.title("Maze")
 
    canvas = Canvas(root, width=len(maze[0]) * 20, height=len(maze) * 20)
    canvas.pack()
 
    for y in range(len(maze)):
        for x in range(len(maze[y])):
            if maze[y][x] == '#':
                canvas.create_rectangle(x * 20, y * 20, (x + 1) * 20, (y + 1) * 20, fill='black')
            elif maze[y][x] == ' ':
                canvas.create_rectangle(x * 20, y * 20, (x + 1) * 20, (y + 1) * 20, fill='white')
 
    root.mainloop()
 
if __name__ == '__main__':
    maze = create_maze(10, 10)  # Générer le labyrinthe
    generate_stl_from_maze(maze)  # Générer le fichier STL du labyrinthe
 
    maze_ros = MazeFactoryROS()  # Initialiser l'insertion du modèle dans Gazebo
    maze_ros.insert_maze_model('maze.stl')  # Insérer le modèle de labyrinthe dans Gazebo
 
    display_maze_with_tkinter(maze)  # Afficher le labyrinthe avec Tkinter
