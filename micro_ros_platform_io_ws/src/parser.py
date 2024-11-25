import struct
import sys

#This parser only parser the "reporting data frame format (in target mode, i.e. NOT engineer mode)"

def parse_ld2410_data(hex_data):

    try:
        # Convert hex string to bytes
        data = bytes.fromhex(hex_data)
    except ValueError as e:
        raise ValueError(f"Invalid hex input: {e}")

    # Step 1: Validate the frame header
    if data[:4] != b'\xF4\xF3\xF2\xF1':
        raise ValueError("Invalid frame header")

    # Step 2: Extract the intra-frame length (2 bytes)
    intra_frame_length = struct.unpack('<H', data[4:6])[0]

    # Validate the intra-frame length
    if intra_frame_length != 0x0D:  # Expected length is 13 bytes
        raise ValueError(f"Unexpected intra-frame length: {intra_frame_length}")

    # Step 3: Validate the data type
    data_type = data[6]
    if data_type != 0x02:  # Expected data type is '2' (Target basic information)
        raise ValueError(f"Unexpected data type: {data_type}")

    # Step 4: Validate the intra-frame header
    if data[7] != 0xAA:
        raise ValueError("Invalid intra-frame header")

    # Step 5: Extract target data
    target_state = data[8]  # 1 byte (see Table 12)
    movement_distance = struct.unpack('<H', data[9:11])[0]  # 2 bytes
    movement_energy = data[11]  # 1 byte
    stationary_distance = struct.unpack('<H', data[12:14])[0]  # 2 bytes
    stationary_energy = data[14]  # 1 byte
    detection_distance = struct.unpack('<H', data[15:17])[0]  # 2 bytes

    # Step 6: Validate the end frame
    if data[-4:] != b'\xF8\xF7\xF6\xF5':
        raise ValueError("Invalid end frame")

    # Return the parsed data
    return {
        "Target State": target_state,
        "Movement Distance (cm)": movement_distance,
        "Movement Energy": movement_energy,
        "Stationary Distance (cm)": stationary_distance,
        "Stationary Energy": stationary_energy,
        "Detection Distance (cm)": detection_distance
    }

# Entry point for the script
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python parser.py '<hex_data>'")
        sys.exit(1)

    hex_data = sys.argv[1]

    try:
        parsed_data = parse_ld2410_data(hex_data)
        print("Parsed Data:")
        for key, value in parsed_data.items():
            print(f"{key}: {value}")
    except Exception as e:
        print(f"Error parsing data: {e}")

#Resultaat test:
#Movement cm waarde klopt niet heeft de waarde van Dection
#Stationary cm waarde klopt niet heeft de waarde van movement cm.
