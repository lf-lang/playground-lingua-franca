### File for AI Pac-Man moves 
# Benjamin Asch

import sys
import hbpacman as pacman

# class Movefinders:
    #TODO: make so doesn't look in opposite direction of past move
def closestpillpath(layout, ghosts, x, y, blocks):
    paths = []
    def pathfinder(layout, ghosts, x, y, blocks, temp = []):
        oncurrpath = False
        if len(temp) < 10:
            for name, change in possiblepacmoves(layout, ghosts, x, y).items():
                if len(temp) < 1 or not (change[0] * -1 == temp[len(temp) - 1][0] and change[1] * -1 == temp[len(temp) - 1][1]):
                    new_x = x + change[0]
                    new_y = y + change[1]
                    for block in blocks:
                        x_condition = x >= block.rect.left - block.rect.width and x <= block.rect.left + block.rect.width
                        y_condition = y >= block.rect.top - block.rect.width and y <= block.rect.bottom + block.rect.width
                        if x_condition and y_condition:
                            temp.append(change)
                            paths.append(temp)
                            oncurrpath = True
                            break
                    if oncurrpath:
                        oncurrpath = False
                        break
                    else:
                        pathfinder(layout, ghosts, new_x, new_y, blocks, [*temp, change])
        else:
            paths.append(temp)
    pathfinder(layout, ghosts, x, y, blocks)
    print(paths, " is paths")
    return min(paths, key=len)

    #TODO: makemore efficient by changing wall list to save by section
    # and only check nearby walls
def possiblepacmoves(layout, ghosts, x, y, cond=True):
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
                print(wall)
                wall_left = wall[0]
                wall_right = wall_left + wall[2]
                wall_top = wall[1]
                wall_bottom = wall_top + wall[3]
                new_x = x + change[0]
                new_y = y + change[1]
                print(wall_top, wall_bottom, new_y)
                condition_x = (new_x <= 0 and new_x >= 606) or (new_x >= wall_left - 12 and new_x <= wall_right + 12) 
                condition_y = (new_y <= 0 and new_y >= 606) or (new_y >= wall_top - 12 and new_y <= wall_bottom + 12)
                print(condition_x, condition_y)
                if condition_x and condition_y:
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
        print(possible, " is possible")
        return possible
#TODO: make more efficient by stopping once length of single path htis threshold
def closeghostdist(layout, ghosts, x, y, threshold):
    paths = []
    def ghostfinder(layout, ghosts, x, y, temp = []):
        oncurrpath = False
        if len(temp) <= threshold:
            for name, change in possiblepacmoves(layout, ghosts, x, y, False).items():
                if len(temp) < 1 or not (change[0] * -1 == temp[len(temp) - 1][0] and change[1] * -1 == temp[len(temp) - 1][1]):
                    new_x = x + change[0]
                    new_y = y + change[1]
                    for ghost in ghosts:
                        x_condition = x >= ghost.value.rect.left - 15 and x <= ghost.value.rect.left + 15
                        y_condition = y >= ghost.value.rect.top - 15 and y <= ghost.value.rect.bottom + 15
                        if x_condition and y_condition:
                            temp.append(name)
                            paths.append(temp)
                            oncurrpath = True
                            break
                    if oncurrpath:
                        oncurrpath = False
                        break
                    else:
                        ghostfinder(layout, ghosts, new_x, new_y, [*temp, name])
    ghostfinder(layout, ghosts, x, y)
    if len(paths) == 0:
        return sys.maxsize
    return min(paths, key=len)
        
