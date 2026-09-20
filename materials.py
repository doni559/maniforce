from physics.bodies import ObjectConfig, PhysicalObject
from physics.connections import Joint
from physics.utils import Vector


#Material Configs
polygon_cfg = ObjectConfig(
    stiffnes_cf=1000,
    density=1,
    friction_cf=0.3,
    restitution=0.4,
    gravity=True,
    draw_trajectory=False
)
box_cfg = ObjectConfig(
    stiffnes_cf=250,
    density=1,
    friction_cf=0.2,
    restitution=0.2,
    gravity=True,
    draw_trajectory=False
)
rubber_ball = ObjectConfig(

    stiffnes_cf=1000,
    density=1,
    friction_cf=0.6,
    restitution=0.2,
    gravity=True,
    draw_trajectory=False
)

#Templates
polygon = PhysicalObject(init_fields=None, config=polygon_cfg,
    corners = 
    [
        Vector(-150, -20),
        Vector(150, -20),
        Vector(150, 20),
        Vector(-300, 20),
    ], start_pos=Vector(0,0), start_velocity=Vector(0,0), start_angle=90, collider_type="Polygon", name="Polygon0")
box = PhysicalObject(init_fields=None, config= box_cfg,corners = 
    [
        Vector(-50, -50),
        Vector(50, -50),
        Vector(50, 50),
        Vector(-50, 50),
    ], start_pos=Vector(0,0), start_velocity=Vector(0,0), start_angle=90, collider_type="Polygon", name="Box0")
ball= PhysicalObject(init_fields=None, config=rubber_ball,    radius=25, start_pos=Vector(0,0), start_velocity=Vector(0,0), collider_type="Circle", name="Ball0")
rope = Joint(anchor_0 = ball, anchor_1=Vector(ball.pos.x+100, ball.pos.y+100), stiffness_cf =100, nodes_count= 10, joint_mass=0.2, damping_cf=1, friction_cf = 0.001)

