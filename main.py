from pygame import *
from pygame.time import Clock
from time import perf_counter,sleep

from physics.bodies import PhysicalObject, Obstacle
from physics.utils import VerletIntegrator, EulerIntegrator

from configs.scenes_manager import Scene, load_scene
from configs.settings import HEIGHT, WIDTH, FPS, PHYSICS_DT, MIN_UNRENDERED_BUFFER_SIZE, MAX_BUFFER_SIZE, SUBSTEPS
from configs.renderer import Renderer
from configs.utils import MainParams, event_handler, ControlEvents

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

def physics_worker(scene: Scene, snapshots_buffer: Queue, **kwargs):
    wait = False
    loaded_scene= Scene(**scene.get_fields())
    prev_time=perf_counter()
    stop_event = kwargs["stop_event"]
    save_event = kwargs["save_event"]
    reload_event = kwargs["reload_event"]
    pause_event = kwargs["pause_event"]

    while not stop_event.is_set():
        cur_buffer_size=snapshots_buffer.qsize()
        if save_event.is_set():
            loaded_scene.save_scene()
            save_event.clear()
        if reload_event.is_set():
            loaded_scene = loaded_scene.restart()
            try:
                while cur_buffer_size > 0:
                    snapshots_buffer.get_nowait()
            except:
                pass
            reload_event.clear()

        if (cur_buffer_size < MIN_UNRENDERED_BUFFER_SIZE or wait == False) and not pause_event.is_set():
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
        else:
            sleep(0.2)
        if snapshots_buffer.full():
            wait= True
    snapshots_buffer.close()
    return

def main():  
    init()

    display.set_mode((WIDTH, HEIGHT))

    screen = display.get_surface()
    clock = time.Clock()

    snapshot_buffer = Queue(maxsize=MAX_BUFFER_SIZE)
    renderer = Renderer(snapshot_buffer)
    ux= UX(50)

    
    screen_borders = [
        Obstacle(0,WIDTH, HEIGHT, HEIGHT+100),
        Obstacle(0,WIDTH, -100, 0),
        Obstacle(-100, 0, 0, HEIGHT),
        Obstacle(WIDTH, WIDTH+100, 0, HEIGHT),
    ]

    scene_to_load = "pendulum_cart_test"
    scene = Scene(load=True, name=scene_to_load, obstacles=screen_borders, substeps=SUBSTEPS, used_integrator=VerletIntegrator())

    params = MainParams()
    control = ControlEvents()

    physics_process = Process(target=physics_worker, args=(scene, snapshot_buffer, ), kwargs={**control.__dict__})
    physics_process.start()


    while params.run:
        try:
            if params.pause == False:
                control.pause_event.clear()
                if not control.reload_event.is_set():
                    renderer.render(screen, params.camera_pos, params.camera_zoom)
            else:
                control.pause_event.set()

            ux.display_fps(screen, clock)
            display.flip()
            clock.tick(FPS)

            params.camera_zoom*=euler**params.camera_zooming
            params.camera_pos[0]+=params.camera_moving[0]
            params.camera_pos[1]+=params.camera_moving[1]

            event_handler(params, control)

        except Exception as e:
            print(e)
            params.run = False

    control.stop_event.set()
    physics_process.kill()
    physics_process.join()
    quit()

if __name__ == "__main__":
    main()