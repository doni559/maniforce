from pygame import *
from pygame.time import Clock

from scenes import scene
from settings import HEIGHT, WIDTH

from typing import Tuple

class UX():
    def __init__(self,font_size: int):
        self.font = font.SysFont(name="FontDefault", size=font_size)

    def draw_text(self,text : str, position: Tuple[int], color: Tuple[int], bg: Tuple[int], screen : Surface):
        screen.blit(self.font.render(text, False, color, bg), position)
        
    def display_fps(self, screen : Surface, clock: Clock):
        self.draw_text(str(int(clock.get_fps())), (30,30), (0,0,0), (255,255,255), screen)

def main():  
    init()
    run= True
    pause = False

    display.set_mode((WIDTH, HEIGHT))

    screen = display.get_surface()
    clock = time.Clock()
    ux= UX(50)


    while run:
        if not pause:
            dt= min(0.1, clock.get_time()/1000)
            scene.render_scene(screen, clock, dt)
            ux.display_fps(screen, clock)
        for e in event.get():
            if e.type == KEYDOWN:
                if e.key == K_ESCAPE:   
                    run=False
                if e.key == K_SPACE: 
                    pause = not pause
            if e.type == QUIT:
                run = False
    quit()

if __name__ == "__main__":
    main()