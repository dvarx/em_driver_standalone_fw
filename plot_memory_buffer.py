filename="adc_readout.txt"
import matplotlib.pyplot as plt
import struct

def read_float32_from_ascii(file_path):
    """
    Reads a series of ASCII-represented hexadecimal bytes from a file,
    converts them into their corresponding byte values, and then interprets
    these bytes as a series of 32-bit floating-point numbers.

    Args:
        file_path (str): The path to the ASCII file.

    Returns:
        list: A list of floating-point numbers.
    """
    float_values = []
    with open(file_path, 'r') as f:
        hex_string = f.read().replace(" ", "").replace("\n", "")

    # Ensure the string length is a multiple of 8 (4 bytes per float, 2 hex chars per byte)
    if len(hex_string) % 8 != 0:
        raise ValueError("The hexadecimal string does not represent a whole number of FLOAT32 values.")

    # Process the hexadecimal string in chunks of 8 characters (4 bytes)
    for i in range(0, len(hex_string), 8):
        byte_string = hex_string[i:i+8]
        # Convert hex string to bytes
        byte_data = bytes.fromhex(byte_string)
        # Unpack bytes as a single-precision float (FLOAT32)
        # The '<f' format string specifies little-endian byte order for a float
        # Change to '>f' for big-endian if necessary
        float_val = struct.unpack('<f', byte_data)[0]
        float_values.append(float_val)

    return float_values

vals=[]

vals=read_float32_from_ascii("adc_readout.txt")

plt.plot(vals)
plt.show()