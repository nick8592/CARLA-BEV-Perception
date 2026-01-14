import sys
import os
import glob
import numpy as np
import carla
import queue
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive display

# Automatically find and add CARLA Python library path
try:
    sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'linux-x86_64'))[0])
except IndexError:
    pass

class BEVLaneMapper:
    def __init__(self, debug_mode=True, enable_visualization=True):
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
        self.overhead_queue = queue.Queue()
        
        # 6. Debug mode
        self.debug_mode = debug_mode
        self.frame_count = 0
        self.total_points_written = 0
        
        # 7. Visualization setup
        self.enable_visualization = enable_visualization
        self.all_points_x = []
        self.all_points_y = []
        if self.enable_visualization:
            plt.ion()  # Enable interactive mode
            self.fig, (self.ax_cam, self.ax_overhead, self.ax_bev) = plt.subplots(1, 3, figsize=(24, 8))
            self.scatter = None
            self.vehicle_marker = None
            self.current_image = None
            self.current_lane_mask = None
            self.current_overhead = None

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
        
        # Spawn RGB BEV camera attached to vehicle
        if self.enable_visualization:
            bev_rgb_bp = self.blueprint_library.find('sensor.camera.rgb')
            bev_rgb_bp.set_attribute('image_size_x', '800')
            bev_rgb_bp.set_attribute('image_size_y', '800')
            bev_rgb_bp.set_attribute('fov', '110')  # Wide FOV for better coverage
            
            # Attach BEV RGB camera to vehicle - positioned high and looking down
            bev_height = 25.0  # Height above vehicle
            bev_transform = carla.Transform(
                carla.Location(x=0.0, y=0.0, z=bev_height),  # Directly above vehicle
                carla.Rotation(pitch=-90, yaw=0, roll=0)  # Looking straight down
            )
            self.overhead_camera = self.world.spawn_actor(bev_rgb_bp, bev_transform, attach_to=self.vehicle)
            self.overhead_camera.listen(self.overhead_queue.put)
            self.actors.append(self.overhead_camera)
            
            # Store parameters for visualization
            self.overhead_fov = 110
            self.overhead_height = bev_height
            print(f">>> RGB BEV camera attached to vehicle at height: {bev_height}m")

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

    def update_visualization(self, v_trans):
        """Update the live triple-view visualization: camera + overhead map + BEV plot"""
        # Clear all subplots
        self.ax_cam.clear()
        self.ax_overhead.clear()
        self.ax_bev.clear()
        
        # LEFT: Camera view with marked road lines
        if self.current_image is not None:
            # Create overlay image
            overlay = self.current_image.copy()
            # Highlight detected lane pixels in bright green
            overlay[self.current_lane_mask] = [0, 255, 0]
            # Blend original and overlay
            alpha = 0.6
            blended = (alpha * overlay + (1 - alpha) * self.current_image).astype(np.uint8)
            
            self.ax_cam.imshow(blended)
            self.ax_cam.set_title(f'Front Camera (Frame {self.frame_count})\nGreen: Detected Road Lines', 
                                 fontsize=11, fontweight='bold')
            self.ax_cam.axis('off')
        
        # CENTER: RGB BEV view from vehicle
        if self.current_overhead is not None:
            self.ax_overhead.imshow(self.current_overhead)
            
            # Draw vehicle marker at center (since camera is attached to vehicle)
            # Vehicle is always at the center of this view
            center_x, center_y = 400, 400
            self.ax_overhead.plot(center_x, center_y, 'r^', markersize=15, 
                                 markeredgecolor='white', markeredgewidth=2, 
                                 label='Vehicle (center)')
            
            # Draw orientation indicator (always pointing up since view rotates with vehicle)
            arrow_length = 30  # pixels
            self.ax_overhead.arrow(center_x, center_y, 0, -arrow_length, 
                                  head_width=15, head_length=15,
                                  fc='red', ec='white', linewidth=2)
            
            # Add scale reference
            fov_rad = np.radians(self.overhead_fov)
            ground_width = 2 * self.overhead_height * np.tan(fov_rad / 2)
            scale_text = f'Coverage: ~{ground_width:.1f}m x {ground_width:.1f}m'
            self.ax_overhead.text(10, 780, scale_text, color='white', fontsize=10,
                                 bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
            
            self.ax_overhead.set_title('RGB Bird\'s-Eye View\n(Moving with Vehicle)', 
                                      fontsize=11, fontweight='bold')
            self.ax_overhead.axis('off')
        
        # RIGHT: BEV plot
        if len(self.all_points_x) > 0:
            # Plot lane points
            self.ax_bev.scatter(self.all_points_x, self.all_points_y, 
                               s=1, c='blue', alpha=0.5, label='Lane Points')
        
        # Plot vehicle position
        self.ax_bev.scatter(v_trans.location.x, v_trans.location.y, 
                           s=200, c='red', marker='^', 
                           label='Vehicle', edgecolors='black', linewidths=2)
        
        # Add vehicle orientation arrow
        yaw = np.radians(v_trans.rotation.yaw)
        arrow_length = 5.0
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        self.ax_bev.arrow(v_trans.location.x, v_trans.location.y, 
                         dx, dy, head_width=2, head_length=2, 
                         fc='red', ec='red', alpha=0.7)
        
        self.ax_bev.set_xlabel('World X (meters)', fontsize=11)
        self.ax_bev.set_ylabel('World Y (meters)', fontsize=11)
        self.ax_bev.set_title(f"BEV Reconstruction\nTotal Points: {len(self.all_points_x)}", 
                             fontsize=11, fontweight='bold')
        self.ax_bev.axis('equal')
        self.ax_bev.grid(True, linestyle='--', alpha=0.3)
        self.ax_bev.legend(loc='upper right', fontsize=9)
        
        plt.tight_layout()
        plt.pause(0.001)  # Brief pause to update display

    def save_final_plot(self):
        """Save final BEV visualization to file"""
        if len(self.all_points_x) == 0:
            print("No points to visualize")
            return
        
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.scatter(self.all_points_x, self.all_points_y, 
                  s=2, c='blue', alpha=0.6, label=f'Lane Points (n={len(self.all_points_x)})')
        
        ax.set_xlabel('World X (meters)', fontsize=14)
        ax.set_ylabel('World Y (meters)', fontsize=14)
        ax.set_title("Bird's-Eye View: Complete Lane Mapping Result", 
                    fontsize=16, fontweight='bold')
        ax.axis('equal')
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.legend(loc='upper right', fontsize=12)
        
        output_file = 'bev_lane_mapping_result.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"\n>>> Saved final visualization to: {output_file}")
        plt.close(fig)

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
                    
                    # Get overhead camera image
                    if self.enable_visualization:
                        try:
                            overhead_image = self.overhead_queue.get(timeout=0.5)
                            overhead_array = np.frombuffer(overhead_image.raw_data, dtype=np.dtype("uint8"))
                            overhead_array = np.reshape(overhead_array, (overhead_image.height, overhead_image.width, 4))
                            self.current_overhead = overhead_array[:, :, :3]  # RGB only
                        except queue.Empty:
                            pass  # Keep previous overhead image
                            
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
                
                # Store for visualization
                if self.enable_visualization:
                    self.current_image = array[:, :, :3]  # RGB channels
                    self.current_lane_mask = lane_mask
                
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
                            
                            # Store points for visualization
                            if self.enable_visualization:
                                self.all_points_x.append(res.x)
                                self.all_points_y.append(res.y)
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
                
                # Update visualization every 10 frames
                if self.enable_visualization and self.frame_count % 10 == 0 and len(self.all_points_x) > 0:
                    self.update_visualization(v_trans)

        except KeyboardInterrupt:
            print("\n>>> User manually stopped.")
        finally:
            self.cleanup()

    def cleanup(self):
        print("\n>>> Releasing resources and closing synchronous mode...")
        print(f">>> Total frames processed: {self.frame_count}")
        print(f">>> Total spatial coordinate points written: {self.total_points_written}")
        
        # Save final visualization
        if self.enable_visualization:
            self.save_final_plot()
            if plt.fignum_exists(self.fig.number):
                plt.close(self.fig)
        
        self.world.apply_settings(self.original_settings)
        if hasattr(self, 'log_file'):
            self.log_file.close()
        
        for actor in self.actors:
            if actor is not None and actor.is_alive:
                actor.destroy()
        print(">>> Completed.")

if __name__ == "__main__":
    mapper = BEVLaneMapper(debug_mode=True, enable_visualization=True)
    mapper.spawn_assets()
    mapper.run()