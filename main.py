import pygame
from env import RRTgraph
from env import RRTmap


def main():
    dimensions = (700, 700)
    # (width,height)
    start = (23, 50)
    goal = (600, 605)
    obsdim = 30
    obsnum = 20
    iteration = 0

    pygame.init()
    mapa = RRTmap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTgraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()
    mapa.drawMap(obstacles)

    while not graph.path_to_goal():
        if iteration % 10 == 0:
            x, y, parents = graph.bias(goal)
            pygame.draw.circle(mapa.map, mapa.grey, (x[-1], y[-1]), mapa.nodeRad + 2, 0)
            pygame.draw.line(mapa.map, mapa.blue, (x[-1], y[-1]), (x[parents[-1]], y[parents[-1]]), mapa.edgethickness)
        else:
            x, y, parents = graph.expand()
            pygame.draw.circle(mapa.map, mapa.grey, (x[-1], y[-1]), mapa.nodeRad + 2, 0)
            pygame.draw.line(mapa.map, mapa.blue, (x[-1], y[-1]), (x[parents[-1]], y[parents[-1]]), mapa.edgethickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1
    mapa.drawpath(graph.GetPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    main()
