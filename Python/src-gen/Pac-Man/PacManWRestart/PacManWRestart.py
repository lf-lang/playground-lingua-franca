import os
import sys
sys.path.append(os.path.dirname(__file__))
# List imported names, but do not use pylint's --extension-pkg-allow-list option
# so that these names will be assumed present without having to compile and install.
from LinguaFrancaPacManWRestart import (  # pylint: disable=no-name-in-module, import-error
    Tag, action_capsule_t, compare_tags, get_current_tag, get_elapsed_logical_time,
    get_elapsed_physical_time, get_logical_time, get_microstep, get_physical_time,
    get_start_time, port_capsule, request_stop, schedule_copy,
    start
)
import LinguaFrancaPacManWRestart as lf
try:
    from LinguaFrancaBase.constants import BILLION, FOREVER, NEVER, instant_t, interval_t
    from LinguaFrancaBase.functions import (
        DAY, DAYS, HOUR, HOURS, MINUTE, MINUTES, MSEC, MSECS, NSEC, NSECS, SEC, SECS, USEC,
        USECS, WEEK, WEEKS
    )
    from LinguaFrancaBase.classes import Make
except ModuleNotFoundError:
    print("No module named 'LinguaFrancaBase'. "
          "Install using \"pip3 install LinguaFrancaBase\".")
    sys.exit(1)
import copy

# From the preamble, verbatim:
import os
curr_dirname = os.path.dirname(__file__)
sys.path.append(curr_dirname)
import hbpacman as pacman
# End of preamble.


# Python class for reactor PacManWRestart
class _PacManWRestart:
    
    # Constructor
    def __init__(self, **kwargs):
        # Define parameters and their default values
        # Handle parameters that are set in instantiation
        self.__dict__.update(kwargs)
        # Define state variables
    @property
    def bank_index(self):
        return self._bank_index # pylint: disable=no-member
    
# Python class for reactor GameController
class _GameController:
    
    # Constructor
    def __init__(self, **kwargs):
        # Define parameters and their default values
        self._number_of_ghosts = 4
        # Handle parameters that are set in instantiation
        self.__dict__.update(kwargs)
        # Define state variables
        self._wall_list = None
        self._gate = None
        self._block_list = pacman.pygame.sprite.RenderPlain()
        self._score_to_win = 0
        self._score = 0
        self._pacman_sprite = None
        self._pacman_collide = pacman.pygame.sprite.RenderPlain()
    @property
    def number_of_ghosts(self):
        return self._number_of_ghosts # pylint: disable=no-member
    
    @property
    def bank_index(self):
        return self._bank_index # pylint: disable=no-member
    def reaction_function_0(self, wall_list, gate):
        
        _all_sprites_list = pacman.pygame.sprite.RenderPlain()
        self._wall_list = pacman.setupRoomOne(_all_sprites_list)
        self._gate = pacman.setupGate(_all_sprites_list)
        
        wall_list.set(self._wall_list)
        gate.set(self._gate)
        
            
        return 0
    def reaction_function_1(self, pacman_sprite):
        
        self._pacman_collide.empty()
        self._pacman_collide.add(pacman_sprite.value)
        self._pacman_sprite = pacman_sprite.value
            
        return 0
    def reaction_function_2(self, block_list):
        
        # Draw the grid
        for row in range(19):
            for column in range(19):
                if (row == 7 or row == 8) and (column == 8 or column == 9 or column == 10):
                    continue
                
                block = pacman.Block(pacman.yellow, 4, 4)
        
                # Set a random location for the block
                block.rect.x = (30*column+6)+26
                block.rect.y = (30*row+6)+26
        
                b_collide = pacman.pygame.sprite.spritecollide(block, self._wall_list, False)
                p_collide = pacman.pygame.sprite.spritecollide(block, self._pacman_collide, False)
                
                if b_collide:
                    continue
                if p_collide:
                    continue
                
                # Add the block to the list of objects
                self._block_list.add(block)
                block_list.set(block) # Send it to be drawn
        # print("Finished drawing blocks")
        self._score_to_win = len(self._block_list)
            
        return 0
    def reaction_function_3(self, pacman_sprite, score, game_over):
        
        blocks_hit_list = pacman.pygame.sprite.spritecollide(self._pacman_sprite, self._block_list, True)
            
        # Check the list of collisions.
        if len(blocks_hit_list) > 0:
            self._score +=len(blocks_hit_list)
        
        if self._score == self._score_to_win:
            game_over.set("Won!")
            request_stop()
        
            
        score.set(self._score)
            
        return 0
    def reaction_function_4(self, ghost_sprites, game_over):
        
        # FIXME: Make this more efficient.
        monsta_list = pacman.pygame.sprite.RenderPlain()
        for ghost in ghost_sprites:
            if ghost.is_present:
                monsta_list.add(ghost.value)
        
        monsta_hit_list = pacman.pygame.sprite.spritecollide(self._pacman_sprite, monsta_list, False)
        
        if monsta_hit_list:
            game_over.set("Lost!")
            request_stop()
        
            
        return 0
    def reaction_function_5(self, tick, block_list):
        
        block_list.set(self._block_list)
            
        return 0
    def reaction_function_6(self):
        
        pacman.pygame.quit()
            
        return 0
# Python class for reactor Player
class _Player:
    
    # Constructor
    def __init__(self, **kwargs):
        # Define parameters and their default values
        self._width = 0
        self._height = 0
        self._image = "images/Trollman.png"
        self._character_class = pacman.Player
        # Handle parameters that are set in instantiation
        self.__dict__.update(kwargs)
        # Define state variables
        self.character_instance = None
        self._wall_list = None
        self._gate_list = None
    @property
    def width(self):
        return self._width # pylint: disable=no-member
    
    @property
    def height(self):
        return self._height # pylint: disable=no-member
    
    @property
    def image(self):
        return self._image # pylint: disable=no-member
    
    @property
    def character_class(self):
        return self._character_class # pylint: disable=no-member
    
    @property
    def bank_index(self):
        return self._bank_index # pylint: disable=no-member
    def reaction_function_0(self, icon_name):
        
        dirname = os.path.dirname(__file__)
        icon_name.set(os.path.join(dirname, self.image))
            
        return 0
    def reaction_function_1(self, icon, sprite):
        
        self.character_instance = self.character_class(self.width, self.height, icon.value)
        sprite.set(self.character_instance)
            
        return 0
    def reaction_function_2(self, wall_list, gate_list):
        
        self._wall_list = wall_list.value
        self._gate_list = gate_list.value
            
        return 0
    def reaction_function_3(self, sprite):
        
        keyboard_events = pacman.pygame.event.get()
        for event in keyboard_events:
            if event.type == pacman.pygame.QUIT:
                request_stop()
            if event.type == pacman.pygame.KEYDOWN:
                if event.key == pacman.pygame.K_LEFT:
                    self.character_instance.changespeed(-30, 0)
                if event.key == pacman.pygame.K_RIGHT:
                    self.character_instance.changespeed(30, 0)
                if event.key == pacman.pygame.K_UP:
                    self.character_instance.changespeed(0, -30)
                if event.key == pacman.pygame.K_DOWN:
                    self.character_instance.changespeed(0, 30)
        
            if event.type == pacman.pygame.KEYUP:
                if event.key == pacman.pygame.K_LEFT:
                    self.character_instance.changespeed(30, 0)
                if event.key == pacman.pygame.K_RIGHT:
                    self.character_instance.changespeed(-30, 0)
                if event.key == pacman.pygame.K_UP:
                    self.character_instance.changespeed(0, 30)
                if event.key == pacman.pygame.K_DOWN:
                    self.character_instance.changespeed(0, -30)
        
        self.character_instance.update(
            self._wall_list,
            self._gate_list
        )
        sprite.set(self.character_instance)
            
        return 0
# Python class for reactor Ghost
class _Ghost:
    
    # Constructor
    def __init__(self, **kwargs):
        # Define parameters and their default values
        self._width = 0
        self._height = 0
        self._image = "images/Trollman.png"
        self._character_class = pacman.Player
        self._directions = ()
        # Handle parameters that are set in instantiation
        self.__dict__.update(kwargs)
        # Define state variables
        self.character_instance = None
        self._wall_list = None
        self._gate_list = None
        self.turn = 0
        self.steps = 0
    @property
    def width(self):
        return self._width # pylint: disable=no-member
    
    @property
    def height(self):
        return self._height # pylint: disable=no-member
    
    @property
    def image(self):
        return self._image # pylint: disable=no-member
    
    @property
    def character_class(self):
        return self._character_class # pylint: disable=no-member
    
    @property
    def directions(self):
        return self._directions # pylint: disable=no-member
    
    @property
    def bank_index(self):
        return self._bank_index # pylint: disable=no-member
    def reaction_function_0(self, icon_name):
        
        dirname = os.path.dirname(__file__)
        icon_name.set(os.path.join(dirname, self.image))
            
        return 0
    def reaction_function_1(self, icon, sprite):
        
        self.character_instance = self.character_class(self.width, self.height, icon.value)
        sprite.set(self.character_instance)
            
        return 0
    def reaction_function_2(self, wall_list, gate_list):
        
        self._wall_list = wall_list.value
        self._gate_list = gate_list.value
            
        return 0
    def reaction_function_3(self, tick, sprite):
        
        returned = self.character_instance.changespeed(
            self.directions,
            False,
            self.turn,
            self.steps,
            len(self.directions)-1
        )
        self.turn = returned[0]
        self.steps = returned[1]
        self.character_instance.changespeed(
            self.directions,
            False,
            self.turn,
            self.steps,
            len(self.directions)-1
        )
        self.character_instance.update(
            self._wall_list,
            False
        )
        sprite.set(self.character_instance)
            
        return 0



# Python class for reactor Display
class _Display:
    
    # Constructor
    def __init__(self, **kwargs):
        # Define parameters and their default values
        self._num_moving_sprites = 0
        self._num_static_sprites = 0
        self._nav_icon = "images/pacman.png"
        # Handle parameters that are set in instantiation
        self.__dict__.update(kwargs)
        # Define state variables
        self._screen = None
        self._font = None
        self._clock = None
        self._static_sprites = pacman.pygame.sprite.RenderPlain()
        self._top_corner_text = None
    @property
    def num_moving_sprites(self):
        return self._num_moving_sprites # pylint: disable=no-member
    
    @property
    def num_static_sprites(self):
        return self._num_static_sprites # pylint: disable=no-member
    
    @property
    def nav_icon(self):
        return self._nav_icon # pylint: disable=no-member
    
    @property
    def bank_index(self):
        return self._bank_index # pylint: disable=no-member
    def reaction_function_0(self):
        
        dirname = os.path.dirname(__file__)
        pacman_icon=pacman.pygame.image.load(os.path.join(dirname, self.nav_icon))
        pacman.pygame.display.set_icon(pacman_icon)
        
        self._clock = pacman.pygame.time.Clock()        
        # Create an 606x606 sized screen
        self._screen = pacman.pygame.display.set_mode([606, 606])
        # Set the title of the window
        pacman.pygame.display.set_caption("Pacman")
        # Create a surface we can draw on
        background = pacman.pygame.Surface(self._screen.get_size())
        # Used for converting color maps and such
        background = background.convert()          
        # Fill the screen with a black background
        background.fill(pacman.black)
        pacman.pygame.font.init()
        self._font = pacman.pygame.font.Font("freesansbold.ttf", 24)
        self._screen.fill(pacman.black)
        
            
        return 0
    def reaction_function_1(self, icon_name, icon):
        
        for (idx, name) in enumerate(icon_name):
            if name.is_present:
                icon[idx].set(pacman.pygame.image.load(name.value).convert())
            
        return 0
    def reaction_function_2(self, tick):
        
        pacman.pygame.display.flip()
        self._clock.tick()
        tick.set(True)
            
        return 0
    def reaction_function_3(self, static_sprites):
        
        for sprite in static_sprites:
            if sprite.is_present and isinstance(sprite.value, pacman.pygame.sprite.Group):
                self._static_sprites.add(sprite.value.sprites())
            elif isinstance(sprite.value, pacman.pygame.sprite.Sprite):
                self._static_sprites.add(sprite.value)
        
        self._static_sprites.draw(self._screen)
            
        return 0
    def reaction_function_4(self, score):
        
        self._top_corner_text=self._font.render("Score: "+str(score.value), True, pacman.red)
        self._screen.blit(self._top_corner_text, [10, 10])
            
        return 0
    def reaction_function_5(self, moving_sprites):
        
        self._screen.fill(pacman.black)
        sprite_list = pacman.pygame.sprite.RenderPlain()
        
        for sprite in moving_sprites:
            if sprite.is_present and isinstance(sprite.value, pacman.pygame.sprite.Group):
                sprite.value.draw(self._screen)
            elif isinstance(sprite.value, pacman.pygame.sprite.Sprite):
                sprite_list.add(sprite.value)
        
        sprite_list.draw(self._screen)
        self._static_sprites.draw(self._screen)
        self._screen.blit(self._top_corner_text, [10, 10])
            
        return 0
    def reaction_function_6(self, game_over):
        
        #Grey background
        w = pacman.pygame.Surface((400,200))  # the size of your rect
        w.set_alpha(10)                # alpha level
        w.fill((128,128,128))           # this fills the entire surface
        self._screen.blit(w, (100,200))    # (0,0) are the top-left coordinates
        
        #Won or lost
        text1=self._font.render(game_over.value, True, pacman.white)
        self._screen.blit(text1, [235, 233])
        
        # text2=font.render("To play again, press ENTER.", True, white)
        # screen.blit(text2, [135, 303])
        # text3=font.render("To quit, press ESCAPE.", True, white)
        # screen.blit(text3, [165, 333])
        
        pacman.pygame.display.flip()
            
        return 0


# Instantiate classes
pacmanwrestart_lf = [None] * 1
pacmanwrestart_controller_lf = [None] * 1
pacmanwrestart_player_lf = [None] * 1
pacmanwrestart_pinky_lf = [None] * 1
pacmanwrestart_blinky_lf = [None] * 1
pacmanwrestart_inky_lf = [None] * 1
pacmanwrestart_clyde_lf = [None] * 1
pacmanwrestart_display_lf = [None] * 1
# Start initializing PacManWRestart of class PacManWRestart
for pacmanwrestart_i in range(1):
    bank_index = pacmanwrestart_i
    pacmanwrestart_lf[0] = _PacManWRestart(
        _bank_index = 0,
    )
    # Start initializing PacManWRestart.controller of class GameController
    for pacmanwrestart_controller_i in range(1):
        bank_index = pacmanwrestart_controller_i
        pacmanwrestart_controller_lf[0] = _GameController(
            _bank_index = 0,
            _number_of_ghosts=4,
        )
    # Start initializing PacManWRestart.player of class Player
    for pacmanwrestart_player_i in range(1):
        bank_index = pacmanwrestart_player_i
        pacmanwrestart_player_lf[0] = _Player(
            _bank_index = 0,
            _width=pacman.w,
            _height=pacman.p_h,
            _image="images/pacman.png",
            _character_class=pacman.Player,
        )
    # Start initializing PacManWRestart.pinky of class Ghost
    for pacmanwrestart_pinky_i in range(1):
        bank_index = pacmanwrestart_pinky_i
        pacmanwrestart_pinky_lf[0] = _Ghost(
            _bank_index = 0,
            _width=pacman.w,
            _height=pacman.m_h,
            _image="images/Pinky.png",
            _character_class=pacman.Ghost,
            _directions=pacman.Pinky_directions,
        )
    # Start initializing PacManWRestart.blinky of class Ghost
    for pacmanwrestart_blinky_i in range(1):
        bank_index = pacmanwrestart_blinky_i
        pacmanwrestart_blinky_lf[0] = _Ghost(
            _bank_index = 0,
            _width=pacman.w,
            _height=pacman.b_h,
            _image="images/Blinky.png",
            _character_class=pacman.Ghost,
            _directions=pacman.Blinky_directions,
        )
    # Start initializing PacManWRestart.inky of class Ghost
    for pacmanwrestart_inky_i in range(1):
        bank_index = pacmanwrestart_inky_i
        pacmanwrestart_inky_lf[0] = _Ghost(
            _bank_index = 0,
            _width=pacman.i_w,
            _height=pacman.m_h,
            _image="images/Inky.png",
            _character_class=pacman.Ghost,
            _directions=pacman.Inky_directions,
        )
    # Start initializing PacManWRestart.clyde of class Ghost
    for pacmanwrestart_clyde_i in range(1):
        bank_index = pacmanwrestart_clyde_i
        pacmanwrestart_clyde_lf[0] = _Ghost(
            _bank_index = 0,
            _width=pacman.c_w,
            _height=pacman.m_h,
            _image="images/Clyde.png",
            _character_class=pacman.Ghost,
            _directions=pacman.Clyde_directions,
        )
    # Start initializing PacManWRestart.display of class Display
    for pacmanwrestart_display_i in range(1):
        bank_index = pacmanwrestart_display_i
        pacmanwrestart_display_lf[0] = _Display(
            _bank_index = 0,
            _num_moving_sprites=6,
            _num_static_sprites=2,
            _nav_icon="images/pacman.png",
        )


# The main function
def main(argv):
    start(argv)

# As is customary in Python programs, the main() function
# should only be executed if the main module is active.
if __name__=="__main__":
    main(sys.argv)
