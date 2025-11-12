import pygame
import random
import time

class SnakeGame:

    def __init__(self):
        # initialize pygame
        pygame.init()
        # player's score
        self.score = 0
        # set the window
        self.window = pygame.display.set_mode((500, 500))
        # font object: consolas, 20px
        self.font = pygame.font.SysFont("consolas", 20)
        # important colors
        self.blue = pygame.Color(0, 0, 255)
        self.green = pygame.Color(149, 212, 122)
        self.red = pygame.Color(255, 0, 0)
        self.white = pygame.Color(255, 255, 255)
        self.black = pygame.Color(0, 0, 0)
        self.pink = pygame.Color(246, 143, 160)
        self.aqua = pygame.Color(0, 176, 178)
        # set the speed of snake
        self.speed = 15
        # head
        self.head = [100, 50]
        # first snake blocks
        self.snake_blocks = [[100, 50], [90, 50], [80, 50], [70, 50]]
        # default direction of snake
        self.dir = "RIGHT"
        self.next_dir = self.dir 
        # fruit control
        self.eaten = False
        # initial fruit position
        self.fruit_pos = (random.randrange(1, (500//10)) * 10, random.randrange(1, 500//10) * 10)


    def show_score(self):
        """
        Function that shows the score aof the player
        """
        # display the surface object
        score_surface = self.font.render(f"Score: {self.score}", True, self.black)
        # create rectangular object for surface
        score_rect = score_surface.get_rect()
        # display the score
        self.window.blit(score_surface, score_rect)


    def spawn_fruit(self):
        """
        Function that spawns fruit randomly in the window
        """
        # choose a random new spot to spawn
        # we will give a 5pm - 10px area thretshold so it's not very diffult
        self.fruit_pos = (random.randrange(1, (500//10)) * 10, random.randrange(1, 500//10) * 10)
        # spawn in screen
        pygame.draw.rect(self.window, self.pink, pygame.Rect(self.fruit_pos[0], self.fruit_pos[1], 10, 10))

    
    def keyboard(self, key):
        """
        Function that handles keyboard events
        """
        if key == pygame.K_UP:
            self.next_dir = "UP"
        if key == pygame.K_DOWN:
            self.next_dir = "DOWN"
        if key == pygame.K_LEFT:
            self.next_dir = "LEFT"
        if key == pygame.K_RIGHT:
            self.next_dir = "RIGHT"

        # make sure we don't break the snake 
        if self.next_dir == "UP" and self.dir != "DOWN":
            self.dir = "UP"
        if self.next_dir == "DOWN" and self.dir != "UP":
            self.dir = "DOWN"
        if self.next_dir == "LEFT" and self.dir != "RIGHT":
            self.dir = "LEFT"
        if self.next_dir == "RIGHT" and self.dir != "LEFT":
            self.dir = "RIGHT"
        

    def snake_mech(self):
        """
        Function that takes care of the snake's mechanism.
        Since the snake's blocks are 10x10, we'll be adding by tens
        """
        # draw the first fruit
        pygame.draw.rect(self.window, self.pink, pygame.Rect(self.fruit_pos[0], self.fruit_pos[1], 10, 10))
        # increment by tens since it's moving
        self.snake_blocks.insert(0, list(self.head))
        # increment size if we ate fruit
        if  self.head[0] == self.fruit_pos[0] and self.head[1] == self.fruit_pos[1]:
            self.score += 10
            self.eaten = True
        else:
            # remove the last block if we didn't grow enough
            self.snake_blocks.pop()
        
        for pos in self.snake_blocks:
            pygame.draw.rect(self.window, self.aqua, pygame.Rect(pos[0], pos[1], 10, 10))

        # move the snake
        if self.dir == "UP":
            self.head[1] -= 10
        if self.dir == "DOWN":
            self.head[1] += 10
        if self.dir == "LEFT":
            self.head[0] -= 10
        if self.dir == "RIGHT":
            self.head[0] += 10
        
        # check if snake is out of the screen
        if self.head[0] < 0 or self.head[0] > 500-10:
            self.game_over()
        if self.head[1] < 0 or self.head[1] > 500-10:
            self.game_over()

        # touching the snake's body makes you lose :(
        for block in self.snake_blocks[1:]:
            if self.head[0] == block[0] and self.head[1] == block[1]:
                self.game_over()
        
    
    def game_over(self):
        """
        Function that handles when the game is over :(
        """
        game_over_surface = self.font.render("You Lost :(", True, self.red)
        # create a rectangle for the text surface
        game_over_rect = game_over_surface.get_rect()
        # setting the position for the text
        game_over_rect.midtop = (500/2, 500/2)
        # blit will draw the text on the screen
        self.window.blit(game_over_surface, game_over_rect)
        pygame.display.flip()
        # after 2 seconds the program will quit
        time.sleep(2)
        pygame.quit()


    def start(self):
        """
        Main function that runs the whole game
        """
        # initialize the window
        pygame.display.set_caption("MLH Snake Game")
        # control fps
        fps = pygame.time.Clock()
        # for breaking out of the game
        running = True
        # the game has to run until we lose
        while running:
            # fill the window with background color
            self.window.fill(self.green)
            # display score
            self.show_score()
            # check events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    self.keyboard(event.key)
            
            # snake mech
            self.snake_mech()
            # check if fruit was eaten 
            if self.eaten:
                self.spawn_fruit()
                self.eaten = False
            
            # refresh the screen
            pygame.display.update()
            fps.tick(self.speed)


if __name__ == "__main__":
    game = SnakeGame()
    game.start()
