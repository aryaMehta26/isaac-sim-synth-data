import argparse
import sys
import os
import yaml
import omni
from omni.isaac.kit import SimulationApp

# Parse arguments first to configure simulation app needs
parser = argparse.ArgumentParser(description="Isaac Sim Synthetic Data Generation")
parser.add_argument("--config", type=str, default="configs/dataset.yaml", help="Path to config file")
parser.add_argument("--frames", type=int, default=None, help="Override number of frames to generate")
parser.add_argument("--seed", type=int, default=None, help="Override random seed")
args = parser.parse_args()

# Load Config
with open(args.config, 'r') as f:
    config = yaml.safe_load(f)

if args.frames:
    config['num_frames'] = args.frames
if args.seed:
    config['seed'] = args.seed

# Initialize Simulation Application
# We need to do this before importing ommniverse modules that depend on the kit being running
kit_config = {
    "headless": config.get("headless", True),
    "width": 1280,
    "height": 720
}
simulation_app = SimulationApp(kit_config)

# Now we can import our modules
import omni.replicator.core as rep
from build_scene import build_scene
from randomize_scene import register_randomizer
from sensors import setup_sensors
from export_annotations import save_annotations

def main():
    print(f"Starting Generation: {config['dataset_name']}")
    print(f"Frames: {config['num_frames']}, Seed: {config['seed']}")
    
    # 1. Build Scene
    stage = build_scene()
    
    # 2. Setup Sensors
    sensors = setup_sensors(config)
    
    # 3. Register Randomization
    register_randomizer()
    
    # 4. Run Generation Loop
    # Write output directory
    output_dir = config.get("output_dir", "output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Trigger Replicator logic
    # We update the randomization trigger to be OnFrame
    with rep.trigger.on_frame(num_frames=config['num_frames']):
        rep.orchestrator.step()
        
    # Wait for data to generate? Replicator usually handles this via the orchestrator.run() or step()
    # For manual control loop:
    
    for i in range(config['num_frames']):
        rep.orchestrator.step()
        
        # Access sensor data
        frame_data = {}
        if "camera" in sensors:
            cam = sensors["camera"]
            # To get actual data, we need to access the annotator's data property
            # This often requires waiting for the render to complete
            
            frame_data["camera"] = {
                "bbox_2d": cam["bbox_2d"].get_data()
                # RGB and Depth saving would be done using rep.Writer or manually saving image buffers
            }
            
        # Export
        save_annotations(frame_data, output_dir, i)
        print(f"Frame {i+1}/{config['num_frames']} completed.")
        
    print("Generation Complete!")
    simulation_app.close()

if __name__ == "__main__":
    main()
