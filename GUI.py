"""
Temperature Monitor GUI with Smoke Detection

 GUI application that reads temperature data from a Bluetooth
serial connection and displays it in real-time. It includes color scaling based on
temperature, smoke detection, and a fire warning alert when temperature exceeds a defined threshold.

- TEMPERATURE_THRESHOLD: Temperature at which fire warning activates (default: 50.0°C)
- COLOR_START_THRESHOLD: Temperature at which color transition starts (default: 20.0°C)
- SERIAL_PORT: Port name for the Bluetooth device
- BAUDRATE: Communication speed (default: 9600)
    
"""

import tkinter as tk
import serial
import threading
import time
import re
import datetime
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# Configuration constants
SERIAL_PORT = 'COM6'  # Change this to match your Bluetooth device
BAUDRATE = 9600
TIMEOUT = 1.0  # Read timeout in seconds
BYTESIZE = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
TEMPERATURE_THRESHOLD = 50.0  # Temperature in °C that triggers fire warning
COLOR_START_THRESHOLD = 20.0  # Temperature in °C at which color transition begins
POLL_INTERVAL = 0.1  # Time between serial port reads (seconds)
BLINK_INTERVAL = 0.5  # Time between warning flashes (seconds)
WINDOW_SIZE = "400x300"  # Width x Height
BACKGROUND_COLOR = "#F5F5DC"  # Beige
GREEN_COLOR = "#299640"  # Specific green color
WARNING_COLOR = "#FFA500"  # Orange
ALERT_COLOR = "#FF0000"  # Red
TIMESTAMP_COLOR = "#808080"  # Grey
SMOKE_COLOR = "#444444"  # Dark Grey for smoke warning
CONNECTED_COLOR = "#0000FF"  # Blue for connected status
DISCONNECTED_COLOR = "#FF0000"  # Red for disconnected status

# Email configuration
EMAIL_SENDER = "aadityasharma0705@gmail.com"
EMAIL_PASSWORD = "mmcy suls cpxb qral"
EMAIL_RECIPIENT = "aaditya07@vt.edu"
EMAIL_SUBJECT = "Warning: Fire Alert"
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587

class TemperatureMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("Temperature Monitor")
        self.root.geometry(WINDOW_SIZE)
        self.root.resizable(True, True)  # Allow resizing
        self.root.configure(bg=BACKGROUND_COLOR)
        
        # Full screen state variable
        self.fullscreen = False
        
        # Email alert state tracking
        self.above_threshold = False  # Track if temperature is currently above threshold
        
        # Smoke detection state
        self.smoke_detected = False  # Track if smoke is currently detected
        
        # Create main container that will scale with window size
        self.main_container = tk.Frame(root, bg=BACKGROUND_COLOR)
        self.main_container.pack(fill=tk.BOTH, expand=True)
        
        # Create timestamp frame with flexible positioning based on screen mode
        self.timestamp_frame = tk.Frame(self.main_container, bg=BACKGROUND_COLOR)
        self.timestamp_frame.place(relx=0.5, rely=0.25, anchor="center")
        
        # Create timestamp container frame for horizontal layout
        self.timestamp_container = tk.Frame(self.timestamp_frame, bg=BACKGROUND_COLOR)
        self.timestamp_container.pack()
        
        # Create timestamp label
        self.timestamp_label = tk.Label(
            self.timestamp_container,
            text="--.--.---- | --:--:--",
            font=("Helvetica", 10),
            fg=TIMESTAMP_COLOR,
            bg=BACKGROUND_COLOR
        )
        self.timestamp_label.pack(side=tk.LEFT)
        
        # Create connection status indicator (dot)
        self.status_indicator = tk.Label(
            self.timestamp_container,
            text="●",  # Unicode circle character
            font=("Helvetica", 10),
            fg=DISCONNECTED_COLOR,  # Start with disconnected color
            bg=BACKGROUND_COLOR
        )
        self.status_indicator.pack(side=tk.LEFT, padx=(5, 0))  # Add small padding between text and dot

        # Create temperature frame in the center (always exactly centered)
        self.temp_frame = tk.Frame(self.main_container, bg=BACKGROUND_COLOR)
        self.temp_frame.place(relx=0.5, rely=0.5, anchor="center")
        
        # Create temperature display
        self.temp_display = tk.Label(
            self.temp_frame,
            text="--.-°C",
            font=("Helvetica", 48),
            fg=GREEN_COLOR,
            bg=BACKGROUND_COLOR
        )
        self.temp_display.pack()
        
        # Create smoke detection frame between temperature and fire warning
        self.smoke_frame = tk.Frame(self.main_container, bg=BACKGROUND_COLOR)
        self.smoke_frame.place(relx=0.5, rely=0.65, anchor="center")
        
        # Create smoke detection label (initially hidden)
        self.smoke_label = tk.Label(
            self.smoke_frame,
            text="SMOKE DETECTED",
            font=("Helvetica", 16, "bold"),
            fg=SMOKE_COLOR,
            bg=BACKGROUND_COLOR
        )
        # Don't pack yet - will be shown only when needed

        # Create warning frame at the bottom (relative position)
        self.warning_frame = tk.Frame(self.main_container, bg=BACKGROUND_COLOR)
        self.warning_frame.place(relx=0.5, rely=0.75, anchor="center")
        
        # Create warning label (initially hidden)
        self.warning_label = tk.Label(
            self.warning_frame,
            text="🔥 WARNING: FIRE 🔥",
            font=("Helvetica", 18, "bold"),
            fg=ALERT_COLOR,
            bg=BACKGROUND_COLOR
        )
        # Don't pack yet - will be shown only when needed

        # Serial connection
        self.serial_connected = False
        self.current_temp = None  # Initially None (no reading yet)
        self.current_timestamp = None  # Track the timestamp of current reading
        self.warning_visible = False
        self.stop_threads = False

        # Initialize threads
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.warning_thread = threading.Thread(target=self.flash_warning)

        # Add key bindings for full screen toggle
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Escape>", self.end_fullscreen)
        
        # Start serial connection and threads
        self.connect_serial()
        self.serial_thread.daemon = True
        self.warning_thread.daemon = True
        self.serial_thread.start()
        self.warning_thread.start()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def toggle_fullscreen(self, event=None):
        """Toggle between fullscreen and windowed mode"""
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)
        
        # Adjust element positioning based on screen mode
        if self.fullscreen:
            # In fullscreen, bring elements closer to the temperature value
            self.timestamp_frame.place_configure(rely=0.35)  # Closer to center
            self.smoke_frame.place_configure(rely=0.60)      # Adjust for fullscreen
            self.warning_frame.place_configure(rely=0.70)    # Closer to center
        else:
            # In windowed mode, return to original positions
            self.timestamp_frame.place_configure(rely=0.25)
            self.smoke_frame.place_configure(rely=0.65)      # Default position
            self.warning_frame.place_configure(rely=0.75)
            
        return "break"  # Prevent default behavior

    def end_fullscreen(self, event=None):
        """Exit fullscreen mode"""
        if self.fullscreen:
            self.fullscreen = False
            self.root.attributes("-fullscreen", False)
            
            # Return to original positions
            self.timestamp_frame.place_configure(rely=0.25)
            self.smoke_frame.place_configure(rely=0.65)      # Default position
            self.warning_frame.place_configure(rely=0.75)
            
        return "break"  # Prevent default behavior

    def connect_serial(self):
        """Establish connection to the serial port with full configuration"""
        try:
            self.serial = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                bytesize=BYTESIZE,
                parity=PARITY,
                stopbits=STOPBITS,
                timeout=TIMEOUT
            )
            self.serial_connected = True
            print(f"Connected to {SERIAL_PORT} with configuration:")
            print(f"  Baudrate: {BAUDRATE}")
            print(f"  Bytesize: {BYTESIZE}")
            print(f"  Parity: {PARITY}")
            print(f"  Stopbits: {STOPBITS}")
            print(f"  Timeout: {TIMEOUT}s")
            
            # Update status indicator to connected (blue)
            self.root.after(0, self.update_connection_status, True)
        except serial.SerialException as e:
            print(f"Error connecting to {SERIAL_PORT}: {e}")
            print("Running in simulation mode...")
            
            # Update status indicator to disconnected (red)
            self.root.after(0, self.update_connection_status, False)
            
            # Use simulated data for testing if connection fails
            self.root.after(1000, self.simulate_data)

    def update_connection_status(self, connected):
        """Update the connection status indicator"""
        if connected:
            self.status_indicator.config(fg=CONNECTED_COLOR)
        else:
            self.status_indicator.config(fg=DISCONNECTED_COLOR)

    def read_serial_data(self):
        """Thread function to continuously read data from serial port"""
        while not self.stop_threads:
            if self.serial_connected:
                try:
                    if self.serial.in_waiting > 0:
                        line = self.serial.readline().decode('utf-8').strip()
                        self.process_data(line)
                    # No else block - if nothing is read, we keep the previous value
                except Exception as e:
                    print(f"Serial read error: {e}")
                    self.serial_connected = False
                    # Update status indicator to disconnected (red)
                    self.root.after(0, self.update_connection_status, False)
            time.sleep(POLL_INTERVAL)

    def process_data(self, data):
        """Process incoming data and update the display"""
        # Check for smoke indicator ("S" at the end)
        smoke_detected = data.endswith('S')
        
        # Extract numeric data using regex to handle potential malformed input
        match = re.search(r'\d+\.\d+|\d+', data)
        if match:
            try:
                temp = float(match.group())
                # Update the GUI from the main thread with new data flag set to True
                self.root.after(0, lambda: self.update_display(temp, smoke_detected, new_data=True))
            except ValueError:
                print(f"Failed to convert temperature: {data}")

    def update_display(self, temperature, smoke_detected=False, new_data=False):
        """
        Update the temperature display with the new value
        
        Args:
            temperature: The temperature value to display
            smoke_detected: Flag indicating if smoke is detected
            new_data: Flag indicating if this is a new reading (to update timestamp)
        """
        self.current_temp = temperature
        
        # Format temperature to one decimal place
        temp_text = f"{temperature:.1f}°C"
        self.temp_display.config(text=temp_text)
        
        # Update timestamp only if this is new data
        if new_data:
            current_time = datetime.datetime.now()
            self.current_timestamp = current_time  # Store timestamp for email alerts
            timestamp_text = current_time.strftime("%m.%d.%Y | %H:%M:%S")
            self.timestamp_label.config(text=timestamp_text)
        
        # Update color based on temperature
        color = self.get_temperature_color(temperature)
        self.temp_display.config(fg=color)
        
        # Update smoke detection status
        if smoke_detected:
            if not self.smoke_detected:  # Only update if state changed
                self.smoke_label.pack()
                self.smoke_detected = True
        else:
            if self.smoke_detected:  # Only update if state changed
                self.smoke_label.pack_forget()
                self.smoke_detected = False
        
        # Check if temperature exceeds threshold
        if temperature >= TEMPERATURE_THRESHOLD:
            if not self.warning_visible:
                self.warning_label.pack()
                self.warning_visible = True
                self.temp_display.config(font=("Helvetica", 48, "bold"))
            
            # Check if we need to send an email alert (crossing threshold from below)
            if not self.above_threshold and new_data:
                self.above_threshold = True
                # Send email alert in a separate thread to avoid GUI freeze
                email_thread = threading.Thread(target=self.send_email_alert)
                email_thread.daemon = True
                email_thread.start()
        else:
            if self.warning_visible:
                self.warning_label.pack_forget()
                self.warning_visible = False
                self.temp_display.config(font=("Helvetica", 48))
            
            # Reset threshold flag when temperature drops below threshold
            if self.above_threshold:
                self.above_threshold = False

    def send_email_alert(self):
        """Send an email alert when temperature exceeds threshold"""
        try:
            # Create message
            msg = MIMEMultipart()
            msg['From'] = EMAIL_SENDER
            msg['To'] = EMAIL_RECIPIENT
            msg['Subject'] = EMAIL_SUBJECT
            
            # Format the timestamp for the email
            timestamp_str = self.current_timestamp.strftime("%B %d, %Y at %H:%M:%S")

            # Add smoke detection info to email if detected
            smoke_info = "Smoke has been detected." if self.smoke_detected else "No smoke has been detected."
            
            # Email body
            body = f"""WARNING: High Temperature Detected!{smoke_info}

Temperature: {self.current_temp:.1f}°C
Time: {timestamp_str}
{smoke_info}

This temperature exceeds the threshold of {TEMPERATURE_THRESHOLD}°C.
Immediate action may be required.

This is an automated message from your Wireless Sensor Node.

-G3 Aaditya Sharma and Bryce Pollak
"""
            msg.attach(MIMEText(body, 'plain'))
            
            # Connect to SMTP server
            server = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
            server.starttls()  # Secure the connection
            server.login(EMAIL_SENDER, EMAIL_PASSWORD)
            
            # Send email
            text = msg.as_string()
            server.sendmail(EMAIL_SENDER, EMAIL_RECIPIENT, text)
            server.quit()
            
            print(f"Alert email sent to {EMAIL_RECIPIENT}")
            
            # Update GUI to show email was sent (optional visual feedback)
            self.root.after(0, self.confirm_email_sent)
            
        except Exception as e:
            print(f"Failed to send alert email: {e}")

    def confirm_email_sent(self):
        """Optional method to provide visual feedback that email was sent"""
        # Flash the warning label a few times to indicate email was sent
        # This could be replaced with a more subtle indicator if preferred
        for _ in range(3):
            self.warning_label.config(text="🔥 WARNING: FIRE - ALERT SENT 🔥")
            self.root.update()
            time.sleep(0.3)
            self.warning_label.config(text="🔥 WARNING: FIRE 🔥")
            self.root.update()
            time.sleep(0.3)

    def get_temperature_color(self, temperature):
        """Calculate color based on temperature using linear interpolation"""
        if temperature >= TEMPERATURE_THRESHOLD:
            return WARNING_COLOR
            
        # Stay green until COLOR_START_THRESHOLD
        if temperature <= COLOR_START_THRESHOLD:
            return GREEN_COLOR
            
        # Calculate ratio for transition from COLOR_START_THRESHOLD to TEMPERATURE_THRESHOLD
        transition_range = TEMPERATURE_THRESHOLD - COLOR_START_THRESHOLD
        ratio = min(max((temperature - COLOR_START_THRESHOLD) / transition_range, 0), 1)
        
        # Convert hex to RGB
        r1, g1, b1 = int(GREEN_COLOR[1:3], 16), int(GREEN_COLOR[3:5], 16), int(GREEN_COLOR[5:7], 16)
        r2, g2, b2 = int(WARNING_COLOR[1:3], 16), int(WARNING_COLOR[3:5], 16), int(WARNING_COLOR[5:7], 16)
        
        # Interpolate
        r = int(r1 + (r2 - r1) * ratio)
        g = int(g1 + (g2 - g1) * ratio)
        b = int(b1 + (b2 - b1) * ratio)
        
        # Convert back to hex
        return f"#{r:02x}{g:02x}{b:02x}"

    def flash_warning(self):
        """Thread function to handle warning label flashing"""
        visible = True
        while not self.stop_threads:
            if self.warning_visible:
                visible = not visible
                self.root.after(0, lambda v=visible: self.toggle_warning(v))
            time.sleep(BLINK_INTERVAL)

    def toggle_warning(self, visible):
        """Toggle warning label visibility for flashing effect"""
        if visible:
            self.warning_label.config(fg=ALERT_COLOR)
        else:
            self.warning_label.config(fg=BACKGROUND_COLOR)

    def simulate_data(self):
        """Generate simulated temperature data for testing"""
        import random
        
        # Occasionally toggle connection status in simulation mode for demonstration
        if random.random() < 0.05:  # 5% chance to toggle connection status
            self.serial_connected = not self.serial_connected
            self.update_connection_status(self.serial_connected)
        
        # Occasionally skip updating to simulate no data received
        if random.random() < 0.8:  # 80% chance to update
            # Occasionally simulate temperature crossing threshold to test email alerts
            if random.random() < 0.1:  # 10% chance to generate value near threshold
                if self.above_threshold:
                    # Generate temperature below threshold if we're currently above
                    temp = random.uniform(TEMPERATURE_THRESHOLD - 10, TEMPERATURE_THRESHOLD - 0.1)
                else:
                    # Generate temperature above threshold if we're currently below 
                    temp = random.uniform(TEMPERATURE_THRESHOLD, TEMPERATURE_THRESHOLD + 10)
            else:
                # Normal temperature range (more likely to be below threshold)
                temp = random.uniform(15, 55)
            
            # Randomly simulate smoke detection (20% chance)
            smoke_detected = random.random() < 0.2
                
            self.update_display(temp, smoke_detected, new_data=True)
        else:
            # Keep the previous temperature value if we have one
            if self.current_temp is not None:
                # Just refresh the display without updating timestamp
                self.update_display(self.current_temp, self.smoke_detected, new_data=False)
        
        self.root.after(1000, self.simulate_data)

    def on_closing(self):
        """Clean up resources when window is closed"""
        self.stop_threads = True
        if self.serial_connected:
            self.serial.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = TemperatureMonitor(root)
    root.mainloop()