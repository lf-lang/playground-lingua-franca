### File for AI Pac-Man moves 
# -
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).
#
# Additional contributors: Benjamin Asch

import sys

class Movefinders:

    def closestpillpath(self, layout, ghosts, x, y, blocks):
        paths = []
        def pathfinder(layout, ghosts, x, y, blocks, temp = []):
            oncurrpath = False
            for name, change in Movefinders.possiblepacmoves(layout, ghosts, x, y).items():
                new_x = x + change[0]
                new_y = y + change[1]
                for block in blocks:
                    x_condition = x >= block.rect.left - block.rect.width and x <= block.rect.left + block.rect.width
                    y_condition = y >= block.rect.top - block.rect.width and y <= block.rect.bottom + block.rect.width
                    if x_condition and y_condition:
                        temp.append(name)
                        paths.append(temp)
                        oncurrpath = True
                        break
                if oncurrpath:
                    oncurrpath = False
                    break
                else:
                    pathfinder(layout, ghosts, new_x, new_y, blocks, [*temp, name])
        pathfinder(layout, ghosts, x, y, blocks)
        return min(paths, key=len)


    def possiblepacmoves(self, layout, ghosts, x, y, cond=True):
            moves = {
                "LEFT": [30, 0], 
                "RIGHT": [-30, 0], 
                "DOWN": [0, 30], 
                "UP": [0, -30]
                }
            possible = {}
            add = True
            for name, change in moves.items():
                for wall in layout:
                    wall_left = wall[0]
                    wall_right = wall_left + wall[2]
                    wall_top = wall[1]
                    wall_bottom = wall_top + wall[3]
                    new_x = x + change[0]
                    new_y = y + change[1]
                    condition_x = (new_x >= 0 and new_x <= 606 and new_x >= wall_left and new_x <= wall_right) 
                    condition_y = (new_y >= 0 and new_y <= 606 and new_y >= wall_top and new_y <= wall_bottom)
                    if condition_x or condition_y:
                        add = False
                        break
                if cond:
                    for ghost in ghosts:
                        if (x + change[0]) == ghost.rect.left and (y + change[1]) == ghost.rect.top:
                            add = False;
                            break
    
                if add:
                    possible.update({name : change})
                else:
                    add = True
            return possible
    
    def closeghostdist(self, layout, ghosts, x, y):
        paths = []
        def ghostfinder(layout, ghosts, x, y, blocks, temp = []):
            oncurrpath = False
            for name, change in Movefinders.possiblepacmoves(layout, ghosts, x, y, False).items():
                new_x = x + change[0]
                new_y = y + change[1]
                for ghost in ghosts:
                    x_condition = x >= ghost.rect.left - 15 and x <= ghost.rect.left + 15
                    y_condition = y >= ghost.rect.top - 15 and y <= ghost.rect.bottom + 15
                    if x_condition and y_condition:
                        temp.append(name)
                        paths.append(temp)
                        oncurrpath = True
                        break
                if oncurrpath:
                    oncurrpath = False
                    break
                else:
                    ghostfinder(layout, ghosts, new_x, new_y, blocks, [*temp, name])
        ghostfinder(layout, ghosts, x, y)
        return min(map(len, paths))
        
