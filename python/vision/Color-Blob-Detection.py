import cv2
import numpy as np
import time
from datetime import datetime

class ColorBlobDetector:
    def __init__(self):
        self.target_color_hex = "#3B9325"  # Default green color
        self.color_tolerance = 40  # Tolerance for color detection
        self.hue_tolerance = 15  # Hue tolerance (0-179)
        self.saturation_tolerance = 50  # Saturation tolerance (0-255)
        self.value_tolerance = 50  # Value tolerance (0-255)
        self.min_area = 3800  # Minimum area for blob detection (increased for HD)
        self.max_area = 6500  # Maximum area for blob detection (increased for HD)
        self.recording = False
        self.video_writer = None
        self.frame_rate = 24  # Reduced for better stability
        self.frame_time = 1.0 / self.frame_rate
        self.actual_fps = 0  # Track actual FPS
        self.frame_count = 0
        self.fps_start_time = time.time()
        
        # Crop settings
        self.enable_crop = True  # Enable/disable cropping
        self.crop_width = 860    # Crop width
        self.crop_height = 860   # Crop height
        
        # Camera settings for overlay
        self.camera_aperture = "f/11"       # Aperture setting
        self.camera_shutter = "1/4000"      # Shutter speed
        self.camera_iso = "ISO 51200"       # ISO setting
        self.show_camera_info = True        # Show camera settings on overlay
        
    def hex_to_bgr(self, hex_color):
        """Convert hex color to BGR format"""
        hex_color = hex_color.lstrip('#')
        rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        # Convert RGB to BGR for OpenCV
        return (rgb[2], rgb[1], rgb[0])
    
    def crop_center(self, frame):
        """Crop center region of the frame"""
        if not self.enable_crop:
            return frame, 0, 0  # Return original frame with zero offsets
            
        height, width = frame.shape[:2]
        
        # Calculate crop dimensions (ensure they don't exceed frame size)
        crop_w = min(self.crop_width, width)
        crop_h = min(self.crop_height, height)
        
        # Calculate center crop coordinates
        start_x = (width - crop_w) // 2
        start_y = (height - crop_h) // 2
        
        # Crop the frame
        cropped_frame = frame[start_y:start_y + crop_h, start_x:start_x + crop_w]
        
        return cropped_frame, start_x, start_y
    
    def create_color_mask(self, frame, target_bgr, tolerance):
        """Create mask for target color with tolerance"""
        # Convert frame to HSV for better color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Convert target BGR to HSV
        target_bgr_array = np.uint8([[target_bgr]])
        target_hsv = cv2.cvtColor(target_bgr_array, cv2.COLOR_BGR2HSV)[0][0]
        
        # Define range of target color in HSV with separate tolerances
        lower_bound = np.array([
            max(0, target_hsv[0] - self.hue_tolerance),
            max(0, target_hsv[1] - self.saturation_tolerance),
            max(0, target_hsv[2] - self.value_tolerance)
        ], dtype=np.uint8)
        upper_bound = np.array([
            min(179, target_hsv[0] + self.hue_tolerance),
            min(255, target_hsv[1] + self.saturation_tolerance),
            min(255, target_hsv[2] + self.value_tolerance)
        ], dtype=np.uint8)
        
        # Create mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        return mask
    
    def detect_blobs(self, frame, mask, offset_x=0, offset_y=0):
        """Detect blobs in the mask and return contours with areas"""
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area <= area <= self.max_area:
                # Get center point and radius
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                # Adjust coordinates back to original frame if cropped
                center = (int(x + offset_x), int(y + offset_y))
                radius = int(radius)
                
                # Adjust contour coordinates back to original frame
                adjusted_contour = contour + np.array([offset_x, offset_y])
                
                valid_blobs.append({
                    'contour': adjusted_contour,
                    'area': area,
                    'center': center,
                    'radius': radius
                })
        
        return valid_blobs
    
    def draw_detections(self, frame, blobs, target_bgr, crop_info=None):
        """Draw detection results on frame"""
        result_frame = frame.copy()
        
        # Draw crop area outline if cropping is enabled
        if self.enable_crop and crop_info:
            crop_x, crop_y, crop_w, crop_h = crop_info
            cv2.rectangle(result_frame, (crop_x, crop_y), 
                         (crop_x + crop_w, crop_y + crop_h), (0, 255, 255), 2)
            cv2.putText(result_frame, f"Crop Area: {crop_w}x{crop_h}", 
                       (crop_x, crop_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        for blob in blobs:
            # Draw contour in target color (thicker for HD)
            cv2.drawContours(result_frame, [blob['contour']], -1, target_bgr, 3)
            
            # Draw blue circle at center (larger for HD)
            cv2.circle(result_frame, blob['center'], 8, (255, 0, 0), -1)
            
            # Draw larger circle outline (thicker for HD)
            cv2.circle(result_frame, blob['center'], blob['radius'], (255, 0, 0), 3)
            
            # Display area text with larger font for HD
            text = f"Area: {blob['area']:.0f}"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            text_x = blob['center'][0] - text_size[0] // 2
            text_y = blob['center'][1] - blob['radius'] - 15
            
            # Add background rectangle for better text visibility
            cv2.rectangle(result_frame, 
                         (text_x - 5, text_y - text_size[1] - 5),
                         (text_x + text_size[0] + 5, text_y + 5),
                         (0, 0, 0), -1)
            
            cv2.putText(result_frame, text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return result_frame
    
    def start_recording(self, frame_width, frame_height):
        """Start video recording with dynamic FPS based on actual processing rate"""
        if not self.recording:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"color_detection_{timestamp}.avi"  # Use .avi for XVID
            
            # Use more compatible codec
            fourcc = cv2.VideoWriter_fourcc(*'XVID')  # XVID codec for better compatibility
            
            # Start with target frame rate, but will be updated dynamically
            initial_fps = max(10, min(30, self.actual_fps if self.actual_fps > 0 else self.frame_rate))
            
            # Create video writer with initial frame rate
            self.video_writer = cv2.VideoWriter(
                filename, 
                fourcc, 
                float(initial_fps),  # Start with estimated FPS
                (frame_width, frame_height)
            )
            
            if not self.video_writer.isOpened():
                print(f"Error: Could not open video writer for {filename}")
                return
                
            self.recording = True
            self.frame_count = 0
            self.fps_start_time = time.time()
            print(f"Started recording: {filename}")
            print(f"Recording at actual processing rate (currently ~{initial_fps:.1f} FPS)")
            print("Recording FPS will adapt to actual processing performance")
    
    def stop_recording(self):
        """Stop video recording"""
        if self.recording and self.video_writer:
            # Calculate actual recording FPS
            recording_duration = time.time() - self.fps_start_time
            actual_recording_fps = self.frame_count / recording_duration if recording_duration > 0 else 0
            
            self.video_writer.release()
            self.video_writer = None
            self.recording = False
            print(f"Recording stopped. Recorded {self.frame_count} frames in {recording_duration:.2f}s")
            print(f"Actual recorded FPS: {actual_recording_fps:.2f}")
            print(f"Processing FPS during recording: {self.actual_fps:.2f}")
            print("Video saved with natural processing timing (no artificial frame rate control)")
    
    def update_target_color(self, hex_color):
        """Update target color from hex string"""
        try:
            self.target_color_hex = hex_color
            print(f"Target color updated to: {hex_color}")
        except ValueError:
            print("Invalid hex color format. Use #RRGGBB format.")
    
    def run(self):
        """Main detection loop"""
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("Error: Could not open camera")
            return
        
        # Set camera resolution to 1920x1080
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # Verify the resolution
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera resolution set to: {actual_width}x{actual_height}")
        
        print("Color Blob Detector Started!")
        print("Controls:")
        print("- Press 'q' to quit")
        print("- Press 'r' to start/stop recording")
        print("- Press 'c' to change target color (in terminal)")
        print("- Press 'h'/'H' to decrease/increase hue tolerance")
        print("- Press 's'/'S' to decrease/increase saturation tolerance")
        print("- Press 'v'/'V' to decrease/increase value tolerance")
        print("- Press '+' to increase color tolerance (all HSV)")
        print("- Press '-' to decrease color tolerance (all HSV)")
        print("- Press 'a' to adjust area thresholds (in terminal)")
        print("- Press 'f' to adjust frame rate (in terminal)")
        print("- Press 'p' to toggle crop mode on/off")
        print("- Press 'o' to adjust crop size (in terminal)")
        print("- Press 'i' to toggle camera info display")
        print("- Press 'k' to adjust camera settings (in terminal)")
        
        last_frame_time = time.time()
        frame_times = []  # Track frame times for FPS calculation
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Frame rate control - more precise timing
            current_time = time.time()
            if current_time - last_frame_time < self.frame_time:
                continue
                
            # Calculate actual FPS
            frame_times.append(current_time)
            if len(frame_times) > 30:  # Keep last 30 frames for FPS calculation
                frame_times.pop(0)
            
            if len(frame_times) > 1:
                avg_frame_time = (frame_times[-1] - frame_times[0]) / (len(frame_times) - 1)
                self.actual_fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
            
            last_frame_time = current_time
            
            # Crop center region if enabled
            if self.enable_crop:
                cropped_frame, offset_x, offset_y = self.crop_center(frame)
                processing_frame = cropped_frame
            else:
                processing_frame = frame
                offset_x, offset_y = 0, 0
            
            # Get target color in BGR
            target_bgr = self.hex_to_bgr(self.target_color_hex)
            
            # Create mask for target color (use cropped frame for processing)
            mask = self.create_color_mask(processing_frame, target_bgr, self.color_tolerance)
            
            # Detect blobs (adjust coordinates back to original frame)
            blobs = self.detect_blobs(processing_frame, mask, offset_x, offset_y)
            
            # Prepare crop info for drawing
            if self.enable_crop:
                crop_info = (offset_x, offset_y, self.crop_width, self.crop_height)
            else:
                crop_info = None
            
            # Draw detections on original full frame
            result_frame = self.draw_detections(frame, blobs, target_bgr, crop_info)
            
            # Add info text with larger font for HD resolution
            frame_height, frame_width = frame.shape[:2]
            crop_status = f"CROP: {self.crop_width}x{self.crop_height}" if self.enable_crop else "CROP: OFF"
            
            # Main info text
            info_text = [
                f"Target Color: {self.target_color_hex}",
                f"HSV Tolerance: H:{self.hue_tolerance}, S:{self.saturation_tolerance}, V:{self.value_tolerance}",
                f"Min Area: {self.min_area}, Max Area: {self.max_area}",
                f"Blobs Found: {len(blobs)}",
                f"Processing FPS: {self.actual_fps:.1f} (Target: {self.frame_rate})",
                f"Resolution: {frame_width}x{frame_height}",
                crop_status
            ]
            
            # Camera settings info (displayed separately)
            camera_info = []
            if self.show_camera_info:
                camera_info = [
                    f"Camera Settings:",
                    f"Aperture: {self.camera_aperture}",
                    f"Shutter: {self.camera_shutter}",
                    f"ISO: {self.camera_iso}"
                ]
            
            if self.recording:
                recording_duration = time.time() - self.fps_start_time
                recorded_fps = self.frame_count / recording_duration if recording_duration > 0 and self.frame_count > 0 else 0
                info_text.append(f"Frames: {self.frame_count}, FPS: {recorded_fps:.1f}")
            
            # Use larger font and spacing for HD resolution
            font_scale = 0.8
            text_thickness = 2
            line_spacing = 35
            
            # Draw main info text (left side)
            for i, text in enumerate(info_text):
                color = (0, 0, 255) if "RECORDING" in text else (255, 255, 255)
                cv2.putText(result_frame, text, (15, 40 + i * line_spacing),
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, text_thickness)
            
            # Draw camera settings (bottom-left corner)
            if camera_info:
                camera_start_y = frame_height - (len(camera_info) * 30 + 20)  # Position from bottom
                
                # Add semi-transparent background for camera info
                overlay = result_frame.copy()
                cv2.rectangle(overlay, (10, camera_start_y - 15), 
                             (350, frame_height - 10), (0, 0, 0), -1)
                cv2.addWeighted(overlay, 0.7, result_frame, 0.3, 0, result_frame)
                
                for i, text in enumerate(camera_info):
                    color = (0, 255, 255) if i == 0 else (255, 255, 255)  # Yellow for title, white for values
                    weight = 2 if i == 0 else 1  # Bold for title
                    cv2.putText(result_frame, text, (20, camera_start_y + i * 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, weight)
            
            # Show mask in larger window for HD resolution  
            mask_width = 320
            mask_height = 240
            
            # Use the processing mask (from cropped frame if enabled)
            display_mask = mask
            if self.enable_crop:
                # Resize cropped mask to fit display area
                display_mask = cv2.resize(mask, (mask_width, mask_height))
            else:
                # For full frame, resize normally
                display_mask = cv2.resize(mask, (mask_width, mask_height))
                
            mask_colored = cv2.cvtColor(display_mask, cv2.COLOR_GRAY2BGR)
            
            # Position mask in top-right corner
            mask_x = frame.shape[1] - mask_width - 15
            mask_y = 15
            result_frame[mask_y:mask_y+mask_height, mask_x:mask_x+mask_width] = mask_colored
            
            mask_title = "Crop Mask" if self.enable_crop else "Color Mask"
            cv2.putText(result_frame, mask_title, (mask_x, mask_y + mask_height + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Record frame if recording (record every processed frame)
            if self.recording and self.video_writer:
                # Record every frame that gets processed (no timing restriction)
                self.video_writer.write(result_frame)
                self.frame_count += 1  # Count recorded frames
            
            # Display frame (resize for comfortable viewing if too large)
            display_frame = result_frame.copy()
            display_height, display_width = display_frame.shape[:2]
            
            # If frame is too large for screen, resize for display only
            max_display_width = 1280
            max_display_height = 720
            
            if display_width > max_display_width or display_height > max_display_height:
                scale = min(max_display_width/display_width, max_display_height/display_height)
                new_width = int(display_width * scale)
                new_height = int(display_height * scale)
                display_frame = cv2.resize(display_frame, (new_width, new_height))
                
            cv2.imshow('Color Blob Detection', display_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('r'):
                if self.recording:
                    self.stop_recording()
                else:
                    self.start_recording(frame.shape[1], frame.shape[0])
            elif key == ord('c'):
                print("\nPause detection. Enter new hex color (e.g., #FF0000):")
                cv2.waitKey(100)  # Small delay
                try:
                    new_color = input("Color: ")
                    if new_color.startswith('#') and len(new_color) == 7:
                        self.update_target_color(new_color)
                    else:
                        print("Invalid format. Use #RRGGBB")
                except:
                    print("Input cancelled or invalid")
            elif key == ord('+') or key == ord('='):
                self.color_tolerance = min(100, self.color_tolerance + 5)
                self.hue_tolerance = min(90, self.hue_tolerance + 3)
                self.saturation_tolerance = min(100, self.saturation_tolerance + 10)
                self.value_tolerance = min(100, self.value_tolerance + 10)
                print(f"All tolerances increased: H±{self.hue_tolerance}, S±{self.saturation_tolerance}, V±{self.value_tolerance}")
            elif key == ord('-'):
                self.color_tolerance = max(5, self.color_tolerance - 5)
                self.hue_tolerance = max(5, self.hue_tolerance - 3)
                self.saturation_tolerance = max(10, self.saturation_tolerance - 10)
                self.value_tolerance = max(10, self.value_tolerance - 10)
                print(f"All tolerances decreased: H±{self.hue_tolerance}, S±{self.saturation_tolerance}, V±{self.value_tolerance}")
            elif key == ord('h'):
                self.hue_tolerance = max(5, self.hue_tolerance - 3)
                print(f"Hue tolerance decreased to: ±{self.hue_tolerance}")
            elif key == ord('H'):
                self.hue_tolerance = min(90, self.hue_tolerance + 3)
                print(f"Hue tolerance increased to: ±{self.hue_tolerance}")
            elif key == ord('s'):
                self.saturation_tolerance = max(10, self.saturation_tolerance - 10)
                print(f"Saturation tolerance decreased to: ±{self.saturation_tolerance}")
            elif key == ord('S'):
                self.saturation_tolerance = min(100, self.saturation_tolerance + 10)
                print(f"Saturation tolerance increased to: ±{self.saturation_tolerance}")
            elif key == ord('v'):
                self.value_tolerance = max(10, self.value_tolerance - 10)
                print(f"Value tolerance decreased to: ±{self.value_tolerance}")
            elif key == ord('V'):
                self.value_tolerance = min(100, self.value_tolerance + 10)
                print(f"Value tolerance increased to: ±{self.value_tolerance}")
            elif key == ord('a'):
                print("\nPause detection. Adjusting area thresholds:")
                cv2.waitKey(100)  # Small delay
                try:
                    new_min = int(input(f"Current min area: {self.min_area}. Enter new min area: "))
                    new_max = int(input(f"Current max area: {self.max_area}. Enter new max area: "))
                    if new_min < new_max:
                        self.min_area = new_min
                        self.max_area = new_max
                        print(f"Area thresholds updated: {self.min_area} - {self.max_area}")
                    else:
                        print("Min area must be less than max area")
                except:
                    print("Input cancelled or invalid")
            elif key == ord('f'):
                print("\nPause detection. Adjusting frame rate:")
                cv2.waitKey(100)  # Small delay
                try:
                    new_fps = int(input(f"Current frame rate: {self.frame_rate}. Enter new FPS (10-60): "))
                    if 10 <= new_fps <= 60:  # Limit range for stability
                        self.frame_rate = new_fps
                        self.frame_time = 1.0 / self.frame_rate
                        print(f"Frame rate updated to: {self.frame_rate} FPS")
                        print("Note: Stop and restart recording to apply new FPS to video file")
                    else:
                        print("Frame rate must be between 10 and 60")
                except:
                    print("Input cancelled or invalid")
            elif key == ord('p'):
                self.enable_crop = not self.enable_crop
                status = "ON" if self.enable_crop else "OFF"
                print(f"Crop mode: {status}")
                if self.enable_crop:
                    print(f"Crop size: {self.crop_width}x{self.crop_height}")
            elif key == ord('o'):
                print("\nPause detection. Adjusting crop size:")
                cv2.waitKey(100)  # Small delay
                try:
                    new_width = int(input(f"Current crop width: {self.crop_width}. Enter new width: "))
                    new_height = int(input(f"Current crop height: {self.crop_height}. Enter new height: "))
                    if new_width > 0 and new_height > 0 and new_width <= 1920 and new_height <= 1080:
                        self.crop_width = new_width
                        self.crop_height = new_height
                        print(f"Crop size updated to: {self.crop_width}x{self.crop_height}")
                    else:
                        print("Invalid crop size. Width and height must be positive and within camera resolution.")
                except:
                    print("Input cancelled or invalid")
            elif key == ord('i'):
                self.show_camera_info = not self.show_camera_info
                status = "ON" if self.show_camera_info else "OFF"
                print(f"Camera info display: {status}")
            elif key == ord('k'):
                print("\nPause detection. Adjusting camera settings:")
                cv2.waitKey(100)  # Small delay
                try:
                    print("Enter camera settings (press Enter to keep current value):")
                    
                    aperture_input = input(f"Aperture (current: {self.camera_aperture}): ").strip()
                    if aperture_input:
                        # Add f/ prefix if not present
                        if not aperture_input.startswith('f/'):
                            aperture_input = f"f/{aperture_input}"
                        self.camera_aperture = aperture_input
                    
                    shutter_input = input(f"Shutter speed (current: {self.camera_shutter}): ").strip()
                    if shutter_input:
                        # Add 1/ prefix for fractional speeds if just number is entered
                        if shutter_input.isdigit() and int(shutter_input) > 10:
                            shutter_input = f"1/{shutter_input}"
                        self.camera_shutter = shutter_input
                    
                    iso_input = input(f"ISO (current: {self.camera_iso}): ").strip()
                    if iso_input:
                        # Add ISO prefix if not present
                        if not iso_input.upper().startswith('ISO'):
                            iso_input = f"ISO {iso_input}"
                        self.camera_iso = iso_input
                    
                    print(f"Camera settings updated:")
                    print(f"  Aperture: {self.camera_aperture}")
                    print(f"  Shutter: {self.camera_shutter}")
                    print(f"  ISO: {self.camera_iso}")
                except:
                    print("Input cancelled or invalid")
        
        # Cleanup
        self.stop_recording()
        cap.release()
        cv2.destroyAllWindows()
        print("Program ended")

if __name__ == "__main__":
    detector = ColorBlobDetector()
    detector.run()
