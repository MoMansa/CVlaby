from pyamaze import maze, agent, COLOR

def DFS(m,start=None):
    if start is None:
        start=(m.rows,m.cols)
    explored=[start]
    frontier=[start]
    dSeacrh=[]
    while len(frontier)>0:
        currCell=frontier.pop()
        dSeacrh.append(currCell)
        if currCell==m._goal:
            break
        poss=0
        for d in 'ENSW':
            if m.maze_map[currCell][d]==True:
                if d =='E':
                    child=(currCell[0],currCell[1]+1)
                if d =='W':
                    child=(currCell[0],currCell[1]-1)
                if d =='N':
                    child=(currCell[0]-1,currCell[1])
                if d =='S':
                    child=(currCell[0]+1,currCell[1])
                if child in explored:
                    continue
                poss+=1
                explored.append(child)
                frontier.append(child)
                
        if poss>1:
            m.markCells.append(currCell)
    
    return dSeacrh


if __name__=='__main__':
        m=maze(6,6)
        m.CreateMaze(6,6,loadMaze='maze--2024-03-12--10-06-14.csv') 
        dSeacrh=DFS(m,(1,1))
        a=agent(m,1,1,footprints=True,)
        m.tracePath({a:dSeacrh},showMarked=True)
    
        m.run()
print(m.maze_map)

