#Pacman in Python with PyGame
#https://github.com/hbokmann/Pacman
#Modified for mine example by Benjamin Asch
  
import pygame
import AIPhosphate as ai
from random import randint
import sys
  
black = (0,0,0)
white = (255,255,255)
blue = (0,0,255)
green = (0,255,0)
gray = (90, 90, 90)
red = (255,0,0)
burgundy = (172, 56, 56)
wash_blue = (0, 102, 204)
filter_orange = (255, 153, 51)
purple = (255,0,255)
yellow   = (255, 255, 0)
brown = (139, 69, 19)
walls = [ [0,0,6,600],
              [0,0,600,6],
              [0,600,606,6],
              [600,0,6,606],
            ]
layout = [ [0,0,6,600],
              [0,0,600,6],
              [0,600,606,6],
              [600,0,6,606]
            ]
minespot = [63, 95]
chargerspot = [63, 515]
washspot = [273, 215]
filterspot = [423, 545]
storespot = [543, 125]

# This class represents the bar at the bottom that the player controls
class Wall(pygame.sprite.Sprite):
    # Constructor function
    def __init__(self,x,y,width,height, color):
        # Call the parent's constructor
        pygame.sprite.Sprite.__init__(self)
  
        # Make a blue wall, of the size specified in the parameters
        self.image = pygame.Surface([width, height])
        self.image.fill(color)
  
        # Make our top-left corner the passed-in location.
        self.rect = self.image.get_rect()
        self.rect.top = y
        self.rect.left = x

# This creates all the walls in room 1
def setupRoomOne(all_sprites_list):
    # Make the walls. (x_pos, y_pos, width, height)
    wall_list=pygame.sprite.RenderPlain()
     
    # This is a list of walls. Each is in the form [x, y, width, height]
    walls = [ [0,0,6,600],
              [0,0,600,6],
              [0,600,606,6],
              [600,0,6,606],
              [300,0,6,66],
              [60,60,186,6],
              [360,60,186,6],
              [60,120,66,6],
              [60,120,6,126],
              [180,120,246,6],
              [300,120,6,66],
              [480,120,66,6],
              [540,120,6,126],
              [120,180,126,6],
              [120,180,6,126],
              [360,180,126,6],
              [480,180,6,126],
              [180,240,6,126],
              [180,360,246,6],
              [420,240,6,126],
              [240,240,42,6],
              [324,240,42,6],
              [240,240,6,66],
              [240,300,126,6],
              [360,240,6,66],
              [0,300,66,6],
              [540,300,66,6],
              [60,360,66,6],
              [60,360,6,186],
              [480,360,66,6],
              [540,360,6,186],
              [120,420,366,6],
              [120,420,6,66],
              [480,420,6,66],
              [180,480,246,6],
              [300,480,6,66],
              [120,540,126,6],
              [360,540,126,6]
            ]
     
    # Loop through the list. Create the wall, add it to the list
    for item in walls:
        wall=Wall(item[0],item[1],item[2],item[3],blue)
        wall_list.add(wall)
        all_sprites_list.add(wall)
         
    # return our new list
    return wall_list

def setupMineWalls(all_sprites_list):
    # Make the walls. (x_pos, y_pos, width, height)
    wall_list=pygame.sprite.RenderPlain()
     
    # This is a list of walls. Each is in the form [x, y, width, height]
    walls = [ [0,0,6,600],
              [0,0,600,6],
              [0,600,606,6],
              [600,0,6,606],
            ]
     
    # Loop through the list. Create the wall, add it to the list
    for item in walls:
        wall=Wall(item[0],item[1],item[2],item[3], gray)
        wall_list.add(wall)
        all_sprites_list.add(wall)
         
    # return our new list
    return wall_list

def setupMineStations(all_sprites_list):
      station_list = pygame.sprite.RenderPlain()

      stations = [ ["mining", burgundy, minespot[0], minespot[1], 30, 30],
                   ["washing", wash_blue, washspot[0], washspot[1], 30, 30],
                   ["filtering", filter_orange, filterspot[0], filterspot[1], 30, 30],
                   ["storing", gray, storespot[0], storespot[1], 30, 30],
                   ["charging", green, chargerspot[0], chargerspot[1], 30, 30]
                 ]
      
      for station in stations:
            place = ActionPlace(station[0], station[1], station[2], station[3], station[4], station[5])
            station_list.add(place)
            all_sprites_list.add(place)
      
      return station_list

def setupGate(all_sprites_list):
      gate = pygame.sprite.RenderPlain()
      gate.add(Wall(282,242,42,2,white))
      all_sprites_list.add(gate)
      return gate

#represents each station
class ActionPlace(pygame.sprite.Sprite):
      
      busy = False
      cargo = []
      
      def __init__(self, name, color, x, y, width, height):
            self.color = color
            pygame.sprite.Sprite.__init__(self)
            self.image = pygame.Surface([width, height])
            self.image.fill(self.color)
            #self.image.set_colorkey(color)
            #pygame.draw.rect(self.image, color, [0, 0, width, height])

            self.rect = self.image.get_rect()
            self.rect.left = x
            self.rect.top = y
            self.name = name
      
      #changes the busy state to True
      def set_busy(self):
          self.busy = True
          self.image.fill(red)
      
      #changes busy state to False
      def not_busy(self):
          self.busy = False
          if self.name == "mining":
              self.image.fill(burgundy)
          elif self.name == "washing":
              self.image.fill(wash_blue)
          elif self.name == "filtering":
              self.image.fill(filter_orange)
          elif self.name == "storing":
              self.image.fill(gray)
          else:
              self.image.fill(green)

      #restarts
      def restart(self):
          self.not_busy()
          self.cargo = []
          
      #changes the color for busyness
      def flip_color(self):
          if self.busy:
              self.color = red
          else:
              if self.name == "mining":
                  self.color = burgundy
              elif self.name == "washing":
                  self.color = wash_blue
              elif self.name == "filtering":
                  self.color = filter_orange
              elif self.name == "storing":
                  self.color = gray
              else:
                  self.color = green
      
      #empties cargo
      def empty_cargo(self):
          self.cargo = []

# This class represents the ball        
# It derives from the "Sprite" class in Pygame
class Block(pygame.sprite.Sprite):
     
    # Constructor. Pass in the color of the block, 
    # and its x and y position
    def __init__(self, color, width, height):
        # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self) 
 
        # Create an image of the block, and fill it with a color.
        # This could also be an image loaded from the disk.
        self.image = pygame.Surface([width, height])
        self.image.fill(white)
        self.image.set_colorkey(white)
        pygame.draw.ellipse(self.image,color,[0,0,width,height])
 
        # Fetch the rectangle object that has the dimensions of the image
        # image.
        # Update the position of this object by setting the values 
        # of rect.x and rect.y
        self.rect = self.image.get_rect() 

#class represents the cargo that the AGV holds
class Cargo(pygame.sprite.Sprite):
      def __init__(self, status):
            self.status = "washing"

# This class represents the bar at the bottom that the player controls
class AGV(pygame.sprite.Sprite):
  
    # Set speed vector
    battery = 100
    wash_cargo = []
    filt_cargo = []
    store_cargo = []
    num_tot_cargo = 0
    total_stored = 0
    change_x=0
    change_y=0
    last_move = [0, 0]
    num_moves = 0
    calcpathmove = 0
    next_moves = []
    eating = False

    # Constructor function
    def __init__(self,x,y, _image):
        # Call the parent's constructor
        pygame.sprite.Sprite.__init__(self)
   
        # Set height, width
        self.image = _image
  
        # Make our top-left corner the passed-in location.
        self.rect = self.image.get_rect()
        self.rect.top = y
        self.rect.left = x
        self.prev_x = x
        self.prev_y = y

    # Clear the speed of the player
    def prevdirection(self):
        self.prev_x = self.change_x
        self.prev_y = self.change_y

    # Change the speed of the player
    def changespeed(self,x,y):
        self.change_x+=x
        self.change_y+=y
        
    # Make the player speed 0
    def speedzero(self):
        self.change_x = 0
        self.change_y = 0
    
    def resetpos(self):
        self.rect.left = w
        self.rect.top = p_h
    # Find a new position for the player
    def update(self,walls,gate = None):
        # Get the old position, in case we need to go back to it
        self.num_moves += 1
        old_x=self.rect.left
        new_x=old_x+self.change_x
        prev_x=old_x+self.prev_x
        self.rect.left = new_x
        
        old_y=self.rect.top
        new_y=old_y+self.change_y
        prev_y=old_y+self.prev_y

        # Did this update cause us to hit a wall?
        x_collide = pygame.sprite.spritecollide(self, walls, False)
        if x_collide:
            # Whoops, hit a wall. Go back to the old position
            self.rect.left=old_x
            # self.rect.top=prev_y
            # y_collide = pygame.sprite.spritecollide(self, walls, False)
            # if y_collide:
            #     # Whoops, hit a wall. Go back to the old position
            #     self.rect.top=old_y
            #     print('a')
        else:

            self.rect.top = new_y

            # Did this update cause us to hit a wall?
            y_collide = pygame.sprite.spritecollide(self, walls, False)
            if y_collide:
                # Whoops, hit a wall. Go back to the old position
                self.rect.top=old_y
                # self.rect.left=prev_x
                # x_collide = pygame.sprite.spritecollide(self, walls, False)
                # if x_collide:
                #     # Whoops, hit a wall. Go back to the old position
                #     self.rect.left=old_x
                #     print('b')

        # if gate != False:
        #   gate_hit = pygame.sprite.spritecollide(self, gate, False)
        #   if gate_hit:
        #     self.rect.left=old_x
        #     self.rect.top=old_y
    #TODO: consolidate the following into one func based on move
    #save potential future moves
    def charge(self, input):
        if input == "full":
            self.battery = 100
        else:
            self.battery += input

    #stores cargo for PhosphateMine.lf example
    def store(self, input):
          self.total_stored += input
    
    #stores processed cargo for BusyMine.lf
    def store_processed(self):
          self.total_stored += len(self.store_cargo)
          self.num_tot_cargo -= len(self.store_cargo)

    #zeros out all cargo for restart
    def zero_cargo(self):
          self.wash_cargo = []
          self.filt_cargo = []
          self.store_cargo = []
          self.num_tot_cargo = 0
          self.total_stored = 0
    #ai_avoid: has ai avoid people
    def ai_avoid(self, layout, people, threshold):
        path = ai.peopleavoid(layout, people, self.rect.left, self.rect.top)
        #print("this is avoid: ", path)
        #print("old x and y " + str(self.rect.left) + ", " + str(self.rect.top))
        self.rect.left += path[0][0]
        self.rect.top += path[0][1]
        #print("new x and y " + str(self.rect.left) + ", " + str(self.rect.top))
        self.last_move = path
        #print("avoid move is: ", self.last_move)
        #print("avoid x y: " + str(self.rect.left) + ", " + str(self.rect.top))
        # if num_moves is not self.num_moves or len(self.next_moves) == 0:
        #     path = ai.allghostavoid(layout, ghosts, self.rect.left, self.rect.top, threshold)
        #     self.rect.left += path[0][0]
        #     self.rect.top += path[0][1]
        #     self.last_move = path[0]
        #     self.next_moves = path[1:]
        # else:
        #     self.rect.left += self.next_moves[0][0]
        #     self.rect.top += self.next_moves[0][1]
        #     self.last_move = self.next_moves[0]
        #     self.next_moves = self.next_moves[1:]
        # path = ai.closeghostdist(layout, ghosts, self.rect.left, self.rect.top, threshold)
        # #TODO: make better solution, take into account other ghosts
        # made_move = False
        # possible = ai.possiblepacmoves(layout, ghosts, self.rect.left, self.rect.top, False)
        # for name, change in possible.items():
        #       if change[0] == path[0][0] * -1 and change[1] == path[0][1] * -1:
        #         self.rect.left += path[0][0] * -1
        #         self.rect.top += path[0][1] * -1
        #         made_move = True
        # #TODO: make so doesnt accidentally bring closer to ghost
        # if not made_move:
        #       if path[0][0] == 0:
        #             if [30, 0] in list(possible.values()):
        #                   self.rect.left += 30
        #             elif [-30, 0] in list(possible.values()):
        #                   self.rect.left += -30
        #       elif path[0][1] == 0:
        #             if [0, 30] in list(possible.values()):
        #                   self.rect.top += 30
        #             elif [0, -30] in list(possible.values()):
        #                   self.rect.top += -30
              # closeghost = ai.closestghost(layout, ghosts, self.rect.left, self.rect.top, threshold)
              # minimum = ai.euclid_dist(self.rect.left, self.rect.top, closeghost[1], closeghost[2])
              # move = [0, 0]
              # for name, change in possible.items():
              #       if ai.euclid_dist(self.rect.left + change[0], self.rect.top + change[1], closeghost[1], closeghost[2]) < minimum:
              #             move = change
              #             minimium = ai.euclid_dist(self.rect.left + change[0], self.rect.top + change[1], closeghost[1], closeghost[2])
              # self.rect.left += move[0]
              # self.rect.top += move[1]
              # random = randint(0, len(possible) - 1)
              # for i, name, change in enumerate(possible.items()):
              #       if i == random:
              #             self.rect.left += change[0]
              #             self.rect.top += change[1]
              # change = list(possible.values())[random]
              # self.rect.left += change[0]
              # self.rect.top += change[1]
              # self.rect.left += possible.values()[random][0]
              # self.rect.top += possible.values()[random][1]
        #print("this is avoid: ", self.last_move)
        self.num_moves += 1

    def stop_eating(self):
          self.eating = False
    def get_num_moves(self):
          return self.num_moves
    def zero_scargo(self):
          self.store_cargo = []
          
    #approach: has ai approach goal_coords location
    def approach(self, layout, people, goal_coords, num_moves):
          if len(self.next_moves) == 0 or num_moves is not self.num_moves: 
          # or self.num_moves + 15 > self.calcpathmove:
            #print("finding approach path")
            path = ai.euclid_approacher(layout, people, self.rect.left + 16, self.rect.top + 16, goal_coords[0], goal_coords[1])
            self.calcpathmove = self.num_moves + 1
            self.rect.left += path[0][0]
            self.rect.top += path[0][1]
            #print("approach move is ", path[0])
            #print("approach x y: " + str(self.rect.left) + ", " + str(self.rect.top))
            self.last_move = path[0]
            self.next_moves = path[1:]
          else:
            self.rect.left += self.next_moves[0][0]
            self.rect.top += self.next_moves[0][1]
            self.last_move = self.next_moves[0]
            #print("approach move is ", self.last_move)
            #print("approach x y: " + str(self.rect.left) + ", " + str(self.rect.top))
            self.next_moves = self.next_moves[1:]
        
          self.num_moves += 1
          

#Inheritime Player klassist
class People(AGV):
    # Change the speed of the ghost
    def changespeed(self,list,ghost,turn,steps,l):
      try:
        z=list[turn][2]
        if steps < z:
          self.change_x=list[turn][0]
          self.change_y=list[turn][1]
          steps+=1
        else:
          if turn < l:
            turn+=1
          elif ghost == "clyde":
            turn = 2
          else:
            turn = 0
          self.change_x=list[turn][0]
          self.change_y=list[turn][1]
          steps = 0
        return [turn,steps]
      except IndexError:
         return [0,0]
     
    def resetpos(self, name):
        self.steps = 0
        self.turn = 0
        self.rect.left = w
        self.rect.top = m_h
        if name == "Clyde":
            self.rect.left = c_w
        elif name == "Inky":
            self.rect.left = i_w
        elif name == "Blinky":
            self.rect.top = b_h
            
    def moveoffgrid(self):
        self.rect.left = 1000
        self.rect.top = 1000
    # def resetdir(self, name):
    #     if name == "Pinky":
    #


Pinky_directions = [
[0,-30,4],
[15,0,9],
[0,15,11],
[-15,0,23],
[0,15,7],
[15,0,3],
[0,-15,3],
[15,0,19],
[0,15,3],
[15,0,3],
[0,15,3],
[15,0,3],
[0,-15,15],
[-15,0,7],
[0,15,3],
[-15,0,19],
[0,-15,11],
[15,0,9]
]

Blinky_directions = [
[0,-15,4],
[15,0,9],
[0,15,11],
[15,0,3],
[0,15,7],
[-15,0,11],
[0,15,3],
[15,0,15],
[0,-15,15],
[15,0,3],
[0,-15,11],
[-15,0,3],
[0,-15,11],
[-15,0,3],
[0,-15,3],
[-15,0,7],
[0,-15,3],
[15,0,15],
[0,15,15],
[-15,0,3],
[0,15,3],
[-15,0,3],
[0,-15,7],
[-15,0,3],
[0,15,7],
[-15,0,11],
[0,-15,7],
[15,0,5]
]

Inky_directions = [
[30,0,2],
[0,-15,4],
[15,0,10],
[0,15,7],
[15,0,3],
[0,-15,3],
[15,0,3],
[0,-15,15],
[-15,0,15],
[0,15,3],
[15,0,15],
[0,15,11],
[-15,0,3],
[0,-15,7],
[-15,0,11],
[0,15,3],
[-15,0,11],
[0,15,7],
[-15,0,3],
[0,-15,3],
[-15,0,3],
[0,-15,15],
[15,0,15],
[0,15,3],
[-15,0,15],
[0,15,11],
[15,0,3],
[0,-15,11],
[15,0,11],
[0,15,3],
[15,0,1],
]

Clyde_directions = [
[-30,0,2],
[0,-15,4],
[15,0,5],
[0,15,7],
[-15,0,11],
[0,-15,7],
[-15,0,3],
[0,15,7],
[-15,0,7],
[0,15,15],
[15,0,15],
[0,-15,3],
[-15,0,11],
[0,-15,7],
[15,0,3],
[0,-15,11],
[15,0,9],
]

# Call this function so the Pygame library can initialize itself
pygame.init()


#default locations for Pacman and monstas
w = 303-16 #Width
p_h = (7*60)+19 #Pacman height
m_h = (4*60)+19 #Monster height
b_h = (3*60)+19 #Binky height
i_w = 303-16-32 #Inky width
c_w = 303+(32-16) #Clyde width


def doNext(message,left,all_sprites_list,block_list,monsta_list,pacman_collide,wall_list,gate):
  while True:
      # ALL EVENT PROCESSING SHOULD GO BELOW THIS COMMENT
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          pygame.quit()
        if event.type == pygame.KEYDOWN:
          if event.key == pygame.K_ESCAPE:
            pygame.quit()
          if event.key == pygame.K_RETURN:
            del all_sprites_list
            del block_list
            del monsta_list
            del pacman_collide
            del wall_list
            del gate
            startGame()

      #Grey background
      w = pygame.Surface((400,200))  # the size of your rect
      w.set_alpha(10)                # alpha level
      w.fill((128,128,128))           # this fills the entire surface
      screen.blit(w, (100,200))    # (0,0) are the top-left coordinates

      #Won or lost
      text1=font.render(message, True, white)
      screen.blit(text1, [left, 233])

      text2=font.render("To play again, press ENTER.", True, white)
      screen.blit(text2, [135, 303])
      text3=font.render("To quit, press ESCAPE.", True, white)
      screen.blit(text3, [165, 333])

      pygame.display.flip()
