import can
from can import Bus, Message

def send_and_receive():
    # Initialize the bus with the PCAN interface and channel
    # 'PCAN_USBBUS1' is the default channel name for the first USB device
    try:
        bus = can.Bus(channel='PCAN_USBBUS1', interface='pcan', bitrate=1000000)
        print(f"Connected to {bus.channel_info}")

        #msg = can.Message(arbitration_id=0x111, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)
        msg = can.Message(arbitration_id=0x111, data=b'87654321', is_extended_id=False)
        try:
            bus.send(msg)
            print(f"Message sent on {bus.channel_info}: {msg}")
        except can.CanError:
            print("Message transmission failed")


    except can.CanInitializationError as e:
        print(f"Initialization failed: {e}")
        print("Please ensure the PEAK driver is installed and the device is connected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    finally:
        # 3. Shutdown the bus
        if 'bus' in locals() and bus is not None:
            bus.shutdown()

if __name__ == "__main__":
    send_and_receive()