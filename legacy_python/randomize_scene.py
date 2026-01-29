import omni.replicator.core as rep

def register_randomizer():
    """Register randomization logic with Replicator."""
    
    # Get lookat target
    # This assumes there's a camera at /World/Camera and it looks at (0,0,0)
    
    with rep.new_layer():
        # Randomize Lighting
        lights = rep.create.light(
            light_type="Sphere",
            intensity=rep.distribution.uniform(200, 800),
            temperature=rep.distribution.normal(6500, 500),
            scale=rep.distribution.uniform(50, 100),
            count=2
        )
        
        with lights:
            rep.modify.pose(
                position=rep.distribution.uniform((-500, -500, 200), (500, 500, 800))
            )

        # Randomize Objects (assuming we have some loaded under /World/Props)
        # This is a placeholder selector
        props = rep.get.prims(path_pattern="/World/Props/.*")
        
        with props:
            rep.modify.pose(
                position=rep.distribution.uniform((-200, -200, 0), (200, 200, 50)),
                rotation=rep.distribution.uniform((0,0,0), (0,0,360))
            )
            # Randomize color
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

def randomize():
    """Trigger the randomization graph."""
    register_randomizer()
    print("Randomization graph registered.")

if __name__ == "__main__":
    randomize()
