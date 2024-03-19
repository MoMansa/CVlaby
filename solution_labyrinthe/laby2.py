from pyamaze import maze, agent, COLOR

if __name__=='__main__':
        m=maze(6,6)
        m.CreateMaze(6,6,loadMaze='maze--2024-03-12--10-06-14.csv') 

        x=[(1,1),(1,2),(2,2),(2,1),(3,1),
                (3,2),(3,3),(4,3),(4,2),(5,2),
                (6,2),(6,1),(5,1),(4,1),(5,1),
                (6,1),(6,2),(6,3),(6,2),(5,2),
                (5,3),(5,2),(4,2),(4,3),(4,4),
                (5,4),(6,4),(6,5),(6,6)]
        b=agent(m,1,1,footprints=True,shape='arrow',color='yellow')

        m.tracePath({b:x})
 
    
        m.run()