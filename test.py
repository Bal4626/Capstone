from dynamixel_sdk import PortHandler, PacketHandler

print("✅ Dynamixel SDK import OK")

# Create a port handler (no hardware needed)
portHandler = PortHandler('/dev/ttyUSB0')
print("PortHandler created:", portHandler is not None)

# Create a packet handler (Protocol 2.0)
packetHandler = PacketHandler(2.0)
print("PacketHandler created:", packetHandler is not None)

