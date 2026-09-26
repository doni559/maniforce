from physics.bodies import ObjectConfig

rubber_ball = ObjectConfig(

    stiffnes_cf=1000,
    density=1,
    friction_cf=0.6,
    restitution=0.2,
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