from pygame import *
from pygame.time import Clock

from configs.scenes_manager import scene
from configs.settings import HEIGHT, WIDTH, FPS
from typing import Tuple

from math import e as euler

class UX():
    def __init__(self,font_size: int):
        self.font = font.SysFont(name="FontDefault", size=font_size)

    def draw_text(self,text : str, position: Tuple[int], color: Tuple[int], bg: Tuple[int], screen : Surface):
        screen.blit(self.font.render(text, True, color, bg), position) 
        
    def display_fps(self, screen : Surface, clock: Clock):
        self.draw_text(f"{str(int(clock.get_fps()))} FPS", ( 10,10), (0,0,0), (255,255,255), screen)

def main():  
    init()
    run= True
    pause = False

    camera_speed= 5
    camera_pos = [0 , 0]
    camera_moving=[0, 0]
    
    camera_zooming = 0
    camera_zoom=euler**0
    zoom_speed=0.005


    display.set_mode((WIDTH, HEIGHT))

    screen = display.get_surface()
    clock = time.Clock()
    ux= UX(50)
    loaded_scene = scene
    

    while run:
        if not pause:
            dt= min(0.1, clock.get_time()/1000)
            loaded_scene.render_scene(screen, dt, camera_pos, camera_zoom)
            ux.display_fps(screen, clock)
            display.flip()
        clock.tick(FPS)

        camera_pos[0]+=camera_moving[0]
        camera_pos[1]+=camera_moving[1]
        camera_zoom*=euler**camera_zooming
        for e in event.get():
            if e.type == KEYDOWN:
                if e.key == K_ESCAPE:   
                    run=False
                if e.key == K_SPACE: 
                    pause = not pause
                if e.key == K_F5:
                    loaded_scene.save_scene()
                if e.key == K_F9:
                    loaded_scene=loaded_scene.restart()

                if e.key == K_w:
                    camera_moving[1]+=camera_speed
                if e.key == K_s:
                    camera_moving[1]-=camera_speed
                if e.key == K_a:
                    camera_moving[0]-=camera_speed
                if e.key == K_d:
                    camera_moving[0]+=camera_speed

                if e.key == K_z:
                    camera_zooming+= zoom_speed
                if e.key == K_x:
                    camera_zooming-= zoom_speed
            if e.type == KEYUP:
                if e.key in (K_w, K_s):
                    camera_moving[1]=0
                if e.key in (K_a, K_d):
                    camera_moving[0]=0
                if e.key in (K_z, K_x):
                    camera_zooming=0
            if e.type == QUIT: 
                run = False
    quit()

if __name__ == "__main__":
    main()