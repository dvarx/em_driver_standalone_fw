filename="adc_readout.txt"
import matplotlib.pyplot as plt

def extract_uint32(hex_string):
    """
    Extracts a 32-bit unsigned integer from a hexadecimal string.

    Args:
        hex_string: A string representing a sequence of hexadecimal bytes,
                    e.g., "85 0A 00 00".

    Returns:
        The 32-bit unsigned integer, or None if the input is invalid.
    """
    try:
        hex_bytes = hex_string.split()  # Split the string into individual hex byte strings
        if len(hex_bytes) != 4:
            return None  # Invalid input: not 4 bytes

        # Convert each hex byte string to an integer
        byte_values = [int(byte, 16) for byte in hex_bytes]

        # Combine the bytes into a 32-bit integer (little-endian)
        uint32_value = (
            (byte_values[3] << 24)
            | (byte_values[2] << 16)
            | (byte_values[1] << 8)
            | byte_values[0]
        )

        return uint32_value

    except ValueError:
        return None  # Invalid hexadecimal format

vals=[]

with open(filename,"r") as fptr:
    for line in fptr:
        linepos=0
        while(linepos<len(line)):
            #read the next 10 characters
            hex_num_str=line[linepos:linepos+11]
            print(hex_num_str)
            vals.append(extract_uint32(hex_num_str))
            linepos+=12

plt.plot(vals)
plt.show()