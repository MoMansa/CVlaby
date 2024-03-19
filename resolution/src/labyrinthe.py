import numpy as np
import random
import tkinter as tk
from stl import mesh

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
    
class MazeView(tk.Canvas):     
	def __init__(self, master, maze, robot_position, cell_size=20, *args, **kwargs):         
		super().__init__(master, *args, **kwargs)         
		self.maze = maze         
		self.cell_size = cell_size
		self.robot_position = robot_position         
		self.draw_maze()  
		self.draw_robot()   
	def draw_maze(self):         
		for row_index, row in enumerate(self.maze):             
			for col_index, cell in enumerate(row):                 
				x0, y0 = col_index * self.cell_size, row_index * self.cell_size                 
				x1, y1 = x0 + self.cell_size, y0 + self.cell_size                 
				if cell == '#':                     
					self.create_rectangle(x0, y0, x1, y1, fill='black')                 
				elif cell == ' ':                     
					self.create_rectangle(x0, y0, x1, y1, fill='white')
	def draw_robot(self):
		x0, y0 = self.robot_position[0] * self.cell_size, self.robot_position[1] * self.cell_size         
		x1, y1 = x0 + self.cell_size, y0 + self.cell_size  
		if self.maze[self.robot_position[1]][self.robot_position[0]] == ' ':     
			self.create_oval(x0, y0, x1, y1, fill='red')

if __name__ == '__main__':
    maze = create_maze(10,10)
    generate_stl_from_maze(maze, output_filename='maze.stl')
    robot_position = (0, 0)
    root = tk.Tk()     
    root.title("Maze")     
    maze_view = MazeView(root, maze, robot_position)     
    maze_view.pack()     
    root.mainloop()

