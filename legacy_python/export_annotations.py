import json
import os
import numpy as np

def convert_to_serializable(obj):
    """Helper to convert numpy types to python native types for JSON serialization."""
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj

def save_annotations(data, output_path, frame_id):
    """Save the collected data for a single frame."""
    
    # Structure the data
    annotation_entry = {
        "frame_id": frame_id,
        "timestamp": 0.0, # Placeholder
        "captures": []
    }
    
    # Process camera data if present
    if "camera" in data:
        cam_data = data["camera"]
        
        # In a real pipeline, we would save images to disk here and reference paths
        # For this script, we'll just save metadata about the bounding boxes
        
        if "bbox_2d" in cam_data:
            bbox_data = cam_data["bbox_2d"] # accessing the data object from the annotator
            
            # Extract bbox info
            # The data format depends on the exact Replicator version, usually: [semanticId, x_min, y_min, x_max, y_max, occlusion_ratio]
            # We'll just dump it raw for now
            
            annotation_entry["captures"].append({
                "sensor": "camera",
                "annotations": {
                    "bounding_box_2d": convert_to_serializable(bbox_data)
                }
            })

    # Append to master JSON file (inefficient for large datasets, but simple for demo)
    # Better approach: write per-frame JSONs and merge later, or use a streaming writer.
    
    master_file = os.path.join(output_path, "annotations.json")
    
    current_data = []
    if os.path.exists(master_file):
        try:
            with open(master_file, 'r') as f:
                current_data = json.load(f)
        except json.JSONDecodeError:
            pass
            
    current_data.append(annotation_entry)
    
    with open(master_file, 'w') as f:
        json.dump(current_data, f, indent=4, default=convert_to_serializable)

