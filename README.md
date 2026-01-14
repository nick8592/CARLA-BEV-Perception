# CARLA Perception Project: BEV Lane Mapper

A Python-based perception system for detecting and mapping road lane markings in bird's-eye view (BEV) coordinates using the CARLA simulator.

## Project Overview

This project leverages CARLA's semantic segmentation camera sensor to detect lane markings from a vehicle-mounted camera perspective and transforms the detected pixels into world coordinates for bird's-eye view visualization and analysis.

### Key Features

- **Real-time Lane Detection**: Uses CARLA's semantic segmentation sensor to identify lane markings
- **Coordinate Transformation**: Converts camera pixel coordinates to world space coordinates
- **BEV Visualization**: Generates bird's-eye view plots showing detected lane positions
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
- Attaches semantic segmentation camera to vehicle
- Detects lane markings (semantic tags 6, 7, 24)
- Records world coordinates to `spatial_records.csv`
- Displays frame-by-frame statistics in debug mode

**Configuration** (in `bev_lane_mapper.py`):
- `cam_h`: Camera height (default: 2.4m)
- `cam_pitch`: Camera pitch angle (default: -20°)
- `f`: Camera focal length (default: 400.0)
- `debug_mode`: Enable detailed console logging
- `show_visualization`: Display real-time visualization (requires GUI)

### 2. Visualize Results

After running the lane mapper, generate a BEV plot:

```bash
python visualize_bev.py
```

This creates `bev_reconstruction.png` showing detected lane points in world coordinates.

## System Design

### Lane Detection Pipeline

1. **Sensor Setup**: Semantic segmentation camera mounted 1.6m forward, 2.4m high, pitched -20°
2. **Frame Processing**: Each frame captured at ~20 FPS in synchronous mode
3. **Semantic Extraction**: Extract semantic segmentation data from camera buffer
4. **Pixel Classification**: Identify pixels matching lane marking tags
5. **Coordinate Transform**: Convert pixel positions to vehicle-relative, then world-relative coordinates
6. **Data Logging**: Record valid world coordinates with timestamp

### Coordinate Systems

- **Camera Frame**: (u, v) pixel coordinates in 800×600 image
- **Vehicle Frame**: (x_rel, y_rel) relative to vehicle position
- **World Frame**: (x, y, z) global CARLA coordinates

Transformation uses:
- Camera intrinsics (focal length, principal point)
- Vehicle rotation (pitch angle)
- Vehicle position and orientation

## Output Files

### spatial_records.csv
CSV file with columns:
- `timestamp`: Simulation timestamp
- `type`: Detection type (currently "lane")
- `world_x`: X coordinate in world frame (meters)
- `world_y`: Y coordinate in world frame (meters)
- `world_z`: Z coordinate in world frame (meters)

### bev_reconstruction.png
Bird's-eye view scatter plot showing all detected lane points projected onto the XY plane.

## Debug Mode

Run with debug enabled for detailed diagnostics:

```python
mapper = BEVLaneMapper(debug_mode=True, show_visualization=False)
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

- Real-time 3D visualization
- Machine learning-based semantic filtering
- Multi-sensor fusion (multiple cameras)
- Batch processing of CARLA recordings
- Web-based visualization dashboard

## License

This project is part of a CARLA perception research initiative.

## Contact

For issues or questions, refer to the CARLA documentation at https://carla.readthedocs.io/
