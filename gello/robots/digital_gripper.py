import time
from typing import Optional

class DigitalGripper:
    """Simple interface for EHPS16A gripper using RTDE for digital output control."""
    
    def __init__(self, rtde_io):
        """Initialize EHS16 gripper connection using existing RTDEIOInterface.
        
        Args:
            rtde_io: Existing RTDEIOInterface instance
        """
        self.rtde_io = rtde_io
        self._connected = True
        
        # Gripper position range (0-255 for compatibility with Robotiq interface)
        self._min_position = 0      # Open position
        self._max_position = 255    # Closed position
        self._current_position = self._min_position
        self._is_closed = False
        
        # Initialize gripper to open position
        self.open()


    def connect(self, hostname: Optional[str] = None, port: Optional[int] = None, 
                socket_timeout: float = 10.0) -> None:
        """Connect to the gripper (kept for compatibility)."""
        self._connected = True

    def disconnect(self) -> None:
        """Disconnect from the gripper."""
        if self._connected:
            self.release()  # Release gripper signals
            self._connected = False

    def open(self):
        if self._is_closed:
            """Open the gripper."""
            # Open gripper: DO[0] = True, DO[2] = False
            self.rtde_io.setStandardDigitalOut(2, False)
            time.sleep(0.01)
            self.rtde_io.setStandardDigitalOut(0, True)
            self._current_position = self._min_position
            self._is_closed = False

    def close(self):
        if not self._is_closed: #only triggers when gripper is open
            """Close the gripper."""
            # Close gripper: DO[2] = True, DO[0] = False
            self.rtde_io.setStandardDigitalOut(0, False)
            time.sleep(0.01)  # Small delay for signal to take effect
            self.rtde_io.setStandardDigitalOut(2, True)
            self._current_position = self._max_position
            self._is_closed = True
    
    def release(self):
        """Release both signals (returns to open position if spring-loaded)."""
        self.rtde_io.setStandardDigitalOut(2, False)
        self.rtde_io.setStandardDigitalOut(0, False)
        time.sleep(0.05)
        self._current_position = self._min_position 
        self._is_closed = False

    def get_current_position(self) -> int:
        """Get the current gripper position (0-255).
        
        Returns:
            int: Current position (0 = open, 255 = closed)
        """
        return self._current_position

    def move(self, position: int, speed: int, force: int) -> tuple:
        """Move gripper to specified position.
        
        Args:
            position: Target position (0-255)
            speed: Not used for EHS16 (kept for compatibility)
            force: Not used for EHS16 (kept for compatibility)
            
        Returns:
            tuple: (success, actual_position)
        """
        # Clip position to valid range
        position = max(self._min_position, min(position, self._max_position))
        
        # Simple threshold-based control
        # If position is less than 50% of range, open; otherwise close
        threshold = (self._min_position + self._max_position) // 2
        if position <= threshold:
            self.open()
        else:
            self.close()
        
        return True, self._current_position
    
# Simple usage example
if __name__ == "__main__":
    # Test the gripper
    import rtde_io
    
    try:
        rtde_io_interface = rtde_io.RTDEIOInterface("192.168.201.101")
        gripper = DigitalGripper(rtde_io_interface)
        
        # Test open/close cycle
        print("Testing EHS16 gripper...")
        
        gripper.open()
        print(f"Position after open: {gripper.get_current_position()}")
        time.sleep(1)
        
        gripper.close()
        print(f"Position after close: {gripper.get_current_position()}")
        time.sleep(1)
        
        # Test move function (compatibility interface)
        success, pos = gripper.move(50, 255, 10)
        print(f"Move to 50: success={success}, position={pos}")
        time.sleep(1)
        
        success, pos = gripper.move(200, 255, 10)
        print(f"Move to 200: success={success}, position={pos}")
        time.sleep(1)
        
        gripper.release()
        print("Gripper released")
        
    finally:
        gripper.disconnect()
        rtde_io_interface.disconnect()