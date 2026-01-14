# CARLA BEV Perception

A Python-based perception system for detecting and mapping road lane markings in bird's-eye view (BEV) coordinates using the CARLA simulator.

## Project Overview

This project leverages CARLA's semantic segmentation camera sensor to detect lane markings from a vehicle-mounted camera perspective and transforms the detected pixels into world coordinates for bird's-eye view visualization and analysis.

### Key Features

- **Real-time Lane Detection**: Uses CARLA's semantic segmentation sensor to identify lane markings
- **Coordinate Transformation**: Converts camera pixel coordinates to world space coordinates
- **RGB BEV View**: Real-time RGB bird's-eye view camera attached to vehicle showing top-down perspective
- **Live Triple-View Visualization**: Simultaneous display of front camera, RGB BEV, and reconstruction plot
- **Vehicle-Centric Display**: BEV reconstruction with vehicle fixed at center (like real BEV systems)
- **Data Logging**: Automatically records spatial coordinates to CSV for further analysis
- **Debug Mode**: Comprehensive logging and diagnostics for troubleshooting
- **Synchronous Simulation**: Enforces synchronized frame stepping for reliable data collection

## Project Structure

```
.
├── bev_lane_mapper.py          # Main lane detection and mapping engine
├── visualize_bev.py            # BEV visualization script
├── requirements.txt            # Python dependencies
├── spatial_records.csv         # Output: detected lane coordinates
├── bev_reconstruction.png      # Output: BEV visualization plot
└── README.md                   # This file
```

## Requirements

### System Requirements
- CARLA 0.9.16 simulator running locally on `localhost:2000`
- Python 3.7+
- Linux/Docker environment

### Python Dependencies

See `requirements.txt`:
- `numpy>=1.21.0` - Numerical computations
- `opencv-python>=4.5.0` - Image processing (cv2)
- `pandas>=1.3.0` - Data manipulation
- `matplotlib>=3.4.0` - Visualization
- `carla>=0.9.16` - CARLA Python API

## Installation

1. **Set up Python environment**:
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Linux/Mac
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Ensure CARLA is running**:
   ```bash
   # In CARLA directory, start the server
   ./CarlaUE4.sh
   ```

## Usage

### 1. Main Lane Detection

Run the lane mapper to start collecting lane coordinate data:

```bash
python bev_lane_mapper.py
```

**Features**:
- Automatically spawns a Tesla Model 3 vehicle with autopilot
- Attaches semantic segmentation camera for lane detection
- Attaches RGB BEV camera (25m above vehicle, looking down)
- Detects lane markings (semantic tag 24 for CARLA 0.9.16)
- Records world coordinates to `spatial_records.csv`
- Displays real-time triple-view visualization:
  - **Left**: Front camera with detected lanes highlighted in green
  - **Center**: RGB BEV view with vehicle at center (moves with vehicle)
  - **Right**: BEV reconstruction plot in vehicle frame (vehicle fixed at center)

**Configuration** (in `bev_lane_mapper.py`):
- `cam_h`: Camera height (default: 2.4m)
- `cam_pitch`: Camera pitch angle (default: -20°)
- `f`: Camera focal length (default: 400.0)
- `debug_mode`: Enable detailed console logging
- `enable_visualization`: Display real-time triple-view visualization (requires GUI)

### 2. Visualize Results

After running the lane mapper, generate a BEV plot:

```bash
python visualize_bev.py
```

This creates `bev_reconstruction.png` showing detected lane points in world coordinates.

## System Design

### Lane Detection Pipeline

1. **Sensor Setup**: 
   - Semantic segmentation camera: 1.6m forward, 2.4m high, pitched -20°
   - RGB BEV camera: 25m above vehicle, pitched -90° (looking straight down)
2. **Frame Processing**: Each frame captured at ~20 FPS in synchronous mode
3. **Semantic Extraction**: Extract semantic segmentation data from camera buffer
4. **Pixel Classification**: Identify pixels matching lane marking tag (24)
5. **Coordinate Transform**: Convert pixel positions to vehicle-relative, then world-relative coordinates
6. **Data Logging**: Record valid world coordinates with timestamp
7. **Visualization Update**: Update triple-view display every 10 frames

### Coordinate Systems

- **Camera Frame**: (u, v) pixel coordinates in 800×600 image
- **Vehicle Frame**: (x_rel, y_rel) relative to vehicle position (used in BEV reconstruction)
- **World Frame**: (x, y, z) global CARLA coordinates (used in data logging)

Transformation uses:
- Camera intrinsics (focal length, principal point)
- Vehicle rotation (pitch, yaw angles)
- Vehicle position and orientation

## Output Files

### spatial_records.csv
CSV file with columns:
- `timestamp`: Simulation timestamp
- `type`: Detection type (currently "lane")
- `world_x`: X coordinate in world frame (meters)
- `world_y`: Y coordinate in world frame (meters)
- `world_z`: Z coordinate in world frame (meters)

### bev_lane_mapping_result.png
Bird's-eye view scatter plot showing all detected lane points projected onto the XY plane (generated at end of session).

## Visualization

The system provides real-time triple-view visualization during execution:

### Left Panel: Front Camera View
- Raw RGB camera view from vehicle perspective
- Detected lane markings highlighted in green overlay
- Shows what the vehicle "sees" in real-time

### Center Panel: RGB BEV View
- Real RGB camera mounted 25m above vehicle looking down
- Vehicle always at center with forward direction indicator
- Coverage: ~114m × 114m ground area (depends on FOV and height)
- View moves and rotates with the vehicle

### Right Panel: BEV Reconstruction
- Accumulated lane detection points in vehicle-centric frame
- Vehicle fixed at center pointing upward
- Blue dots: detected lane points
- Red triangle: vehicle position (always at origin)
- Viewing window: ±50 meters
- Shows transformation of all historical detections relative to current vehicle pose

## Debug Mode

Run with debug enabled for detailed diagnostics:

```python
mapper = BEVLaneMapper(debug_mode=True, enable_visualization=True)
```

Debug output includes:
- Detected semantic tags per frame
- Vehicle position tracking
- Lane pixel detection statistics
- Coordinate transformation results
- Points accepted/rejected per frame

## Troubleshooting

### No lane markings detected
- Verify CARLA server is running: `telnet localhost 2000`
- Check vehicle position (must be on roads with lane markings)
- Verify semantic segmentation is enabled in CARLA world settings

### Camera frames timing out
- Ensure synchronous mode is properly enabled
- Check CARLA frame rate and timeout settings
- Verify vehicle is spawned and sensor is attached

### Coordinate transformation issues
- Check camera intrinsics (focal length, principal point) match your CARLA camera setup
- Verify vehicle transform data is valid
- Review `cam_pitch` angle alignment with your setup

## Performance Notes

- Processing speed: ~20 FPS (synchronous, 0.05s fixed timestep)
- Memory usage: Minimal (single image buffer)
- Sampling: 15 lane pixels per frame logged (configurable)
- Disk I/O: Direct flush to disk for reliability

## Future Enhancements

- [ ] Dynamic BEV height adjustment based on speed
- [ ] Multiple vehicle tracking in BEV view
- [ ] Machine learning-based lane prediction
- [ ] Integration with path planning algorithms
- [ ] Recording and playback of BEV sequences
- [ ] Web-based visualization dashboard

## Recent Updates

- **v0.2** (Jan 2026): Added RGB BEV camera with vehicle-centric reconstruction
  - Replaced fixed overhead camera with vehicle-attached RGB BEV
  - Updated BEV reconstruction to use vehicle-relative frame
  - Added triple-view real-time visualization
- **v0.1** (Initial): Basic lane detection and world coordinate mapping

## License

This project is part of a CARLA perception research initiative.

## Contact

For issues or questions, refer to the CARLA documentation at https://carla.readthedocs.io/
