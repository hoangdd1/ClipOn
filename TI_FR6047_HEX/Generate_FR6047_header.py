import os
import re
import sys

def generate_c_header(input_file, output_file):
    with open(input_file, 'r') as file:
        lines = file.readlines()

    struct_entries = []
    address = None
    data_lines = []

    for line in lines:
        if line.startswith('@'):
            if address is not None:
                # Process previous address and data lines
                data = ' '.join(data_lines).replace(' ', ', 0x')
                data_size = len(data.split(','))
                struct_entries.append((address, data, data_size))
            
            # Start new address and data block
            address = line[1:].strip()
            data_lines = []
        else:
            data_lines.append(line.strip())

    # Process the last address and data block
    if address is not None:
        data = ' '.join(data_lines).replace(' ', ', 0x').replace(', 0xq', '')
        data_size = len(data.split(','))
        struct_entries.append((address, data, data_size))

    with open(output_file, 'w') as file:
        file.write("#ifndef HEADER_FILE_H\n")
        file.write("#define HEADER_FILE_H\n\n")
        file.write("#include <stdint.h>\n\n")
        file.write("typedef struct {\n")
        file.write("    uint16_t base_address;\n")
        file.write("    uint16_t data_size;\n")
        file.write("    uint8_t data[];\n")  # Flexible array member
        file.write("} TI_HEX_DataStruct;\n\n")

        file.write("const TI_HEX_DataStruct TI_HEX_data_array[] = {\n")
        for address, data, data_size in struct_entries:
            file.write("    {0x" + address + ", " + str(data_size) + ", {0x" + data + "}},\n")
        file.write("};\n\n")
        file.write("#endif // HEADER_FILE_H\n")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python ./Generate_FR6047_header.py <input_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = os.path.splitext(input_file)[0] + '.h'
    generate_c_header(input_file, output_file)
    print(f"Header file generated: {output_file}")

