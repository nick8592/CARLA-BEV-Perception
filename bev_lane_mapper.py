import sys
import os
import glob
import numpy as np
import carla
import queue

# Automatically find and add CARLA Python library path
try:
    sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'linux-x86_64'))[0])
except IndexError:
    pass

class BEVLaneMapper:
    def __init__(self, debug_mode=True):
        # 1. Initialize Client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # 2. Enable synchronous mode (prevent frame drops in Docker)
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05 # 20 FPS
        self.world.apply_settings(settings)
        
        self.blueprint_library = self.world.get_blueprint_library()
        self.actors = []
        
        # 3. Projection parameters
        self.cam_h = 2.4     
        self.cam_pitch = -20 # Slightly increased angle for closer lane marking capture
        self.f = 400.0       # Intrinsic focal length
        self.cu, self.cv = 400.0, 300.0
        
        # 4. Data logging
        self.log_file = open("spatial_records.csv", "w", buffering=1)
        self.log_file.write("timestamp,type,world_x,world_y,world_z\n")
        
        # 5. Image queue
        self.image_queue = queue.Queue()
        
        # 6. Debug mode
        self.debug_mode = debug_mode
        self.frame_count = 0
        self.total_points_written = 0

    def spawn_assets(self):
        # Spawn vehicle
        bp = self.blueprint_library.find('vehicle.tesla.model3')
        spawn_point = self.world.get_map().get_spawn_points()[0]
        self.vehicle = self.world.spawn_actor(bp, spawn_point)
        self.vehicle.set_autopilot(True)
        self.actors.append(self.vehicle)

        # Spawn semantic segmentation camera
        sem_bp = self.blueprint_library.find('sensor.camera.semantic_segmentation')
        sem_bp.set_attribute('image_size_x', '800')
        sem_bp.set_attribute('image_size_y', '600')
        sem_bp.set_attribute('fov', '90')
        
        cam_transform = carla.Transform(
            carla.Location(x=1.6, z=self.cam_h), 
            carla.Rotation(pitch=self.cam_pitch)
        )
        self.sensor = self.world.spawn_actor(sem_bp, cam_transform, attach_to=self.vehicle)
        self.sensor.listen(self.image_queue.put)
        self.actors.append(self.sensor)

    def pixel_to_world_coords(self, u, v, v_trans):
            pitch_rad = np.radians(self.cam_pitch)
            y_n = (v - self.cv) / self.f
            x_n = (u - self.cu) / self.f
            
            denom = np.sin(pitch_rad) + y_n * np.cos(pitch_rad)
            if abs(denom) < 1e-6: 
                return None 
            
            dist_forward = self.cam_h * (np.cos(pitch_rad) - y_n * np.sin(pitch_rad)) / denom
            dist_lateral = self.cam_h * x_n / denom
            
            # Reject points behind camera or too far away
            if 0.1 < dist_forward < 200.0:
                relative_loc = carla.Location(x=float(dist_forward), y=float(dist_lateral), z=0.0)
                world_loc = v_trans.transform(relative_loc)
                return world_loc
            
            return None

    def run(self):
        print(">>> System startup. Running diagnostics...")
        print(f">>> Debug mode: {self.debug_mode}")
        
        try:
            while True:
                # Ensure server advances a frame
                frame = self.world.tick()
                self.frame_count += 1
                
                # Use timeout to prevent infinite waiting
                try:
                    image = self.image_queue.get(timeout=2.0)
                except queue.Empty:
                    print(">>> Timeout waiting for image. Check if CARLA is still running")
                    continue

                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                v_trans = self.vehicle.get_transform()
                
                # CARLA semantic segmentation: labels typically in R channel (index 2)
                semantic_data = array[:, :, 2]
                
                # Debug: display all detected semantic tags in first 5 frames
                if self.debug_mode and self.frame_count <= 5:
                    unique_tags = np.unique(semantic_data)
                    print(f"\nFrame {self.frame_count} - Detected semantic tags: {unique_tags}")
                    print(f"   Vehicle position: X={v_trans.location.x:.2f}, Y={v_trans.location.y:.2f}, Z={v_trans.location.z:.2f}")
                
                # Check lane marking label for CARLA 0.9.16
                # Tag 24: RoadLines (confirmed for 0.9.16)
                lane_mask = np.isin(semantic_data, [24])
                
                v_idx, u_idx = np.where(lane_mask)
                
                # Debug: display detection statistics
                if self.debug_mode:
                    print(f"Frame {self.frame_count}: Detected {len(u_idx)} lane marking pixels", end="")
                
                if len(u_idx) > 0:
                    # Reduce sampling density (15 points) to ensure stable writing
                    sample_size = min(15, len(u_idx))
                    indices = np.linspace(0, len(u_idx)-1, sample_size, dtype=int)
                    
                    points_written = 0
                    points_rejected = 0
                    
                    for i in indices:
                        res = self.pixel_to_world_coords(u_idx[i], v_idx[i], v_trans)
                        if res:
                            # Force conversion to float and write
                            line = f"{image.timestamp:.4f},lane,{res.x:.3f},{res.y:.3f},{res.z:.3f}\n"
                            self.log_file.write(line)
                            points_written += 1
                        else:
                            points_rejected += 1
                    
                    self.total_points_written += points_written
                    
                    # Force flush to disk
                    self.log_file.flush()
                    os.fsync(self.log_file.fileno())
                    
                    if self.debug_mode:
                        print(f" -> Written {points_written} points, Rejected {points_rejected} points (Total: {self.total_points_written})")
                else:
                    if self.debug_mode:
                        print(" -> WARNING: No lane markings detected")

        except KeyboardInterrupt:
            print("\n>>> User manually stopped.")
        finally:
            self.cleanup()

    def cleanup(self):
        print("\n>>> Releasing resources and closing synchronous mode...")
        print(f">>> Total frames processed: {self.frame_count}")
        print(f">>> Total spatial coordinate points written: {self.total_points_written}")
        
        self.world.apply_settings(self.original_settings)
        if hasattr(self, 'log_file'):
            self.log_file.close()
        
        for actor in self.actors:
            if actor is not None and actor.is_alive:
                actor.destroy()
        print(">>> Completed.")

if __name__ == "__main__":
    mapper = BEVLaneMapper(debug_mode=True)
    mapper.spawn_assets()
    mapper.run()