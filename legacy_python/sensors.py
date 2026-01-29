import omni.replicator.core as rep

def setup_camera(resolution=(1280, 720)):
    """Create and configure the camera sensor."""
    camera = rep.create.camera(
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        focal_length=35.0,
        focus_distance=400.0,
        horizontal_aperture=20.955,
        clipping_range=(0.1, 100000)
    )
    
    # Render products
    render_product = rep.create.render_product(camera, resolution)
    
    # Annotators
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(render_product)
    
    depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth.attach(render_product)
    
    # Semantic Segmentation
    sem_seg = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
    sem_seg.attach(render_product)
    
    # Bounding Box 2D
    bbox_2d = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
    bbox_2d.attach(render_product)
    
    return {
        "render_product": render_product,
        "rgb": rgb,
        "depth": depth,
        "semantic_segmentation": sem_seg,
        "bbox_2d": bbox_2d
    }

def setup_lidar():
    """Create and configure the LiDAR sensor (physx based)."""
    # Note: Full physics LiDAR often requires the Isaac Sim sensor extension enabled
    # Here we simulate it via Replicator's pointcloud annotator just for the example pathway
    # In a full app, you would use omni.isaac.sensor.LidarRtx
    
    # For now, let's assume we use a secondary camera for depth-based point cloud approximation or similar
    # Or just returning None as a placeholder if extensions aren't fully guaranteed in this context
    pass

def setup_sensors(config):
    """Main entry to setup all sensors based on config."""
    sensors = {}
    
    if "camera" in config:
        sensors["camera"] = setup_camera(tuple(config["camera"].get("resolution", (1280, 720))))
        
    return sensors
