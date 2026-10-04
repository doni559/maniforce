from pygame import *
from pygame.time import Clock
from time import perf_counter

from configs.scenes_manager import scene, Scene, load_scene
from configs.settings import HEIGHT, WIDTH, FPS, PHYSICS_DT, MIN_UNRENDERED_BUFFER_SIZE, MAX_BUFFER_SIZE
from configs.renderer import Renderer

from typing import Tuple

from multiprocessing import Queue, Process, Event


from math import e as euler

class UX():
    def __init__(self,font_size: int):
        self.font = font.SysFont(name="FontDefault", size=font_size)

    def draw_text(self,text : str, position: Tuple[int], color: Tuple[int], bg: Tuple[int], screen : Surface):
        screen.blit(self.font.render(text, True, color, bg), position) 
        
    def display_fps(self, screen : Surface, clock: Clock):
        self.draw_text(f"{str(int(clock.get_fps()))} FPS", ( 10,10), (0,0,0), (255,255,255), screen)

def physics_worker(scene: Scene, snapshots_buffer: Queue, stop_event):
    wait = False
    loaded_scene= Scene(**scene.get_fields())
    prev_time=perf_counter()
    while not stop_event.is_set():
        cur_buffer_size=snapshots_buffer.qsize()
        if cur_buffer_size < MIN_UNRENDERED_BUFFER_SIZE or wait == False:
            if wait == True:
                wait=False
            
            loaded_scene.step(PHYSICS_DT) 
            snapshot = loaded_scene.make_snapshot()
            snapshots_buffer.put(snapshot)
            current_time=perf_counter()
            past_time=(current_time-prev_time)
            prev_time=perf_counter()
            rtf= (PHYSICS_DT)/past_time
        
            print(rtf, cur_buffer_size)
            
        if cur_buffer_size >= MAX_BUFFER_SIZE:
            wait= True
    snapshots_buffer.close()
    return


            



def main():  
    init()
    run= True
    pause = False

    camera_speed= 10
    camera_pos = [0 , 0]
    camera_moving=[0, 0]
    
    camera_zooming = 0
    camera_zoom=euler**0
    zoom_speed=0.01


    display.set_mode((WIDTH, HEIGHT))

    screen = display.get_surface()
    clock = time.Clock()

    snapshot_buffer = Queue(maxsize=MAX_BUFFER_SIZE)
    renderer = Renderer(snapshot_buffer)
    ux= UX(50)

    loaded_scene = scene
    stop_event=Event()

    physics_process = Process(target=physics_worker, args=(loaded_scene, snapshot_buffer, stop_event))
    physics_process.start()


    while run:
        
        renderer.render(screen, camera_pos, camera_zoom)
        ux.display_fps(screen, clock)
        display.flip()
    
        clock.tick(FPS)

        camera_zoom*=euler**camera_zooming
        camera_pos[0]+=camera_moving[0]
        camera_pos[1]+=camera_moving[1]

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
                if e.key == K_f:
                    camera_pos=[0,0]
                    camera_zoom=1

                if e.key == K_w:
                    camera_moving[1]+=camera_speed/camera_zoom
                if e.key == K_s:
                    camera_moving[1]-=camera_speed/camera_zoom
                if e.key == K_a:
                    camera_moving[0]-=camera_speed/camera_zoom
                if e.key == K_d:
                    camera_moving[0]+=camera_speed/camera_zoom

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
    stop_event.set()
    physics_process.kill()
    physics_process.join()
    quit()

if __name__ == "__main__":
    main()