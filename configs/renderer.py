from physics.bodies import PhysicalObject, Obstacle
from physics.connections import Joint
from physics.utils import Vector

from configs.scenes_manager import SceneSnapshot, ObjectSnapshot
from configs.settings import PHYSICS_DT

from pygame import draw, Surface
from pygame.time import Clock

from multiprocessing import Queue
from queue import Empty

from time import perf_counter

from typing import List


class Renderer():
    def __init__(self, buffer: Queue):
        self.snapshot_buffer = buffer

    def render(self, screen:Surface, camera_pos: List[float], camera_zoom: float):
        try:
            snapshot : SceneSnapshot = self.snapshot_buffer.get(block=False)
        except Empty:    
            return
        screen.fill((255, 255, 255))
        
        objects_to_draw= snapshot.objects

        for object in objects_to_draw:
            points=[point.convert_to_screen_cords(camera_pos, camera_zoom) for point in object.points]
            if object.type is Joint:
                points=object.points
                vector= points[1]
                start_pos = points[0]

                vector.draw(screen, start_pos=start_pos, color=(0,0,0), camera_pos=camera_pos, camera_zoom=camera_zoom, width=object.width)
            elif object.type is PhysicalObject:
                color = (0, 255, 0)
                if object.form == "Circle":
                    center = points[0]

                    draw.circle(
                        screen,
                        color=color,
                        center=center,
                        radius=object.radius*camera_zoom,
                    )
                if object.form == "Polygon":
                    draw.polygon(
                        screen,
                        color=color,
                        points=points
                    )
            elif object.type is Obstacle:
                if object.form == "Polygon":
                    draw.polygon(
                        screen,
                        color=(0,0,0),
                        points=points
                    )
            else:
                if object.form == "Circle":
                    draw.circle(
                        screen,
                        color=(255,0,255),
                        center=points[0],
                        radius=object.radius*camera_zoom
                    )
        