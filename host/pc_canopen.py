import canopen
import logging
import time

# Optional: Configure logging to see canopen and python-can debug messages
logging.getLogger('canopen').setLevel(logging.DEBUG)
logging.getLogger('can').setLevel(logging.DEBUG)

def on_tpdo_received(pdo_map):
    #print(f"TPDO received (COB-ID: {pdo_map.cob_id:X}):")
    for variable in pdo_map:
        # Accessing the value using the .raw attribute
        print(f"  {variable.name} = {variable.raw}")

def main():
    # 1. Create a network representing one CAN bus
    network = canopen.Network()

    try:
        # 2. Connect to the CAN bus using a Windows-compatible backend
        # The arguments are passed to python-can's can.interface.Bus() constructor.
        # Example for PEAK-System PCAN interface on Windows:
        # channel: 'PCAN_USBBUS1' (check your driver test application for the exact name/serial number)
        # bustype: 'pcan'
        # bitrate: Set to match your CAN network's bitrate (e.g., 250000)
        network.connect(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
        # Other potential backends include 'kvaser', 'vector', 'ixxat', etc.

        print("Connected to CAN bus successfully.")

        # 3. Add a remote node to the network with its Electronic Data Sheet (EDS) file
        # You need the EDS file provided by the device manufacturer.
        # Replace 'od.eds' with the actual path to your EDS file.
        node_id = 6
        eds_file_path = 'c:\\Users\\ssingh\\fastbit\\stm32u5_mcu2\\host\\e35.eds' # Example EDS file path
        remote_eds_file_path = 'c:\\Users\\ssingh\\fastbit\\stm32u5_mcu2\\host\\DS301_profile.eds' # Example remote EDS file path
        
        # The object dictionary file is required for the library to know how to communicate
        node = network.add_node(node_id, eds_file_path)
        print(f"Node {node_id} added.")

        # 3. Initially in PRE-OPERATIONAL (after bootup)
        # Configure SDOs here if needed
        # node.sdo['Producer Heartbeat Time'].raw = 1000

        # 4. Change state to OPERATIONAL (NMT start)
        # node.nmt.state = 'OPERATIONAL'
        # print(f"Node {node_id} state: {node.nmt.state}")

        # 4. Wait for the node to boot up and transition to operational state
        # Canopen devices typically send a "NMT bootup" message on power up.
        # You might need to send an NMT start command to bring it to operational mode.
        #node.nmt.wait_for_bootup(10) 
        # print(f"Node {node_id} is operational.")
        
        # 5. Read a variable from the device using SDO
        # Access objects by name or index.
        # device_name = node.sdo['Manufacturer device name'].raw
        # print(f"Device name: {device_name}")

        remote_node_id = 1
        remote_node = canopen.RemoteNode(remote_node_id, remote_eds_file_path) #
        network.add_node(remote_node)

        remote_device_type = remote_node.sdo['Device type']
        device_type_value = remote_device_type.raw
        print(f"The device type is 0x{device_type_value:X}") #

        # 6. Write a variable to the device using SDO
        # node.sdo['Producer heartbeat time'].raw = 1000  # Set heartbeat to 1000 ms
        # print("Set heartbeat time to 1000 ms.")
        
        # 7. Use PDOs for faster, real-time data
        # Read PDO configuration from node (needs to be configured first)
        # node.pdo.read() 
        # print("PDO configuration read.")

        remote_node.tpdo.read()
        remote_node.rpdo.read()

        # Re-map TPDO[1]
        remote_node.tpdo[1].clear()
        remote_node.tpdo[1].add_variable('velocity')
        #remote_node.tpdo[1].add_variable('one_more_counter')
        # remote_node.tpdo[1].trans_type = 254
        # remote_node.tpdo[1].event_timer = 1.5
        # remote_node.tpdo[1].enabled = True
        # Save new PDO configuration to node
        remote_node.tpdo[1].save()
        
        # Transmit SYNC every 100 ms
        network.sync.start(0.1)
        
        try:
            remote_node.nmt.state = 'OPERATIONAL'
            # Wait for the state to change
            # remote_node.nmt.wait_for_bootup(10) 
            print("remote_node is now OPERATIONAL")
        except Exception as e:
            print(f"Failed to set node to OPERATIONAL state: {e}")
        
        

        try:
            while True:
                # Keep the script running to listen for messages
                # Read a value from TPDO[1]
                remote_node.tpdo[1].wait_for_reception()
                
                print("Waiting for TPDOs to be received...")
                #counter = remote_node.tpdo[1]['counter'].raw
                #print(f"Received TPDO[1] with counter value: {counter}")
                # tpdo_1 = remote_node.tpdo[1]
                # on_tpdo_received(tpdo_1)
                print(f"Cob id: {remote_node.tpdo[1].cob_id:X}")
                velocity = remote_node.tpdo[1]['velocity'].phys # Use .phys to get the physical value
                print(f"velocity: {velocity}")
                if velocity > 5000:
                    remote_node.sdo['velocity'].raw = 0x00
        except KeyboardInterrupt:
            print("Exiting...")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # 8. Disconnect from the CAN bus gracefully
        network.disconnect()
        print("Disconnected from CAN bus.")

if __name__ == '__main__':
    main()
