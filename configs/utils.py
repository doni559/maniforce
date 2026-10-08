from pygame import *
from dataclasses import dataclass, field

from typing import List
from multiprocessing import Event as event_factory
from multiprocessing.synchronize import Event

from configs.scenes_manager import Scene

@dataclass()
class MainParams():
    camera_pos : List[float] = field(default_factory=lambda : [0,0])
    camera_moving: List[float] = field(default_factory=lambda : [0,0])
    camera_zoom: float = 1
    camera_zooming : float = 0
    camera_speed: float = 10
    zoom_speed: float = 0.01

    run: bool = True
    pause: bool = False

@dataclass
class ControlEvents():
    stop_event: Event = event_factory()
    pause_event: Event = event_factory()
    reload_event: Event = event_factory()
    save_event: Event = event_factory()

def event_handler(params: MainParams, control: ControlEvents):
    for e in event.get():
        if e.type == KEYDOWN:
            if e.key == K_ESCAPE:   
                params.run=False
            if e.key == K_SPACE: 
                params.pause = not params.pause
            if e.key == K_F5:
                control.save_event.set()
            if e.key == K_F9:
                control.reload_event.set()
            if e.key == K_f:
                params.camera_pos=[0,0]
                params.camera_zoom=1
    
            if e.key == K_w:
                params.camera_moving[1]+=params.camera_speed/params.camera_zoom
            if e.key == K_s:
                params.camera_moving[1]-=params.camera_speed/params.camera_zoom
            if e.key == K_a:
                params.camera_moving[0]-=params.camera_speed/params.camera_zoom
            if e.key == K_d:
                params.camera_moving[0]+=params.camera_speed/params.camera_zoom
    
            if e.key == K_z:
                params.camera_zooming+= params.zoom_speed
            if e.key == K_x:
                params.camera_zooming-= params.zoom_speed
        if e.type == KEYUP:
            if e.key in (K_w, K_s):
                params.camera_moving[1]=0
            if e.key in (K_a, K_d):
                params.camera_moving[0]=0
            if e.key in (K_z, K_x):
                params.camera_zooming=0
        if e.type == QUIT:
            params.run = False
