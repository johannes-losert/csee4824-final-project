import os
import sys

def pad_output(input_file, chunk_length):
    # Open the input file in read mode
    with open(input_file, 'r') as file:
        # Read all lines from the input file and store them in a list
        lines = file.readlines()

    chunks = []  # List to store chunks of lines
    chunk = []  # List to store lines in a chunk
    for line in lines:
        if line.startswith('++++'):
            # If a line starts with '++++', it indicates the start of a new chunk
            if chunk:
                # If there is a non-empty chunk, append it to the list of chunks
                chunks.append(chunk)
                chunk = []  # Reset the chunk list
        chunk.append(line)  # Add the line to the current chunk

    if chunk:
        # If there is a non-empty chunk remaining, append it to the list of chunks
        chunks.append(chunk)

    # Get the current working directory
    cwd = os.getcwd()
    # Set the output file path to 'output.txt' in the current working directory
    output_file = os.path.join(cwd, 'output.txt')

    # Open the output file in write mode
    with open(output_file, 'w') as file:
        for chunk in chunks:
            # Calculate the padding needed for each chunk

            num_pad_lines = chunk_length - len(chunk)
            line_no = 0
            
            for line in chunk:
                out_line = "[" + str(line_no) + "] " + line
                line_no += 1
                file.write(line)
            
            if (num_pad_lines > 0):
                for i in range(num_pad_lines):
                    pad_line = "[" + str(line_no) + "] \n"
                    file.write(pad_line)
                    line_no += 1


# Usage example
if len(sys.argv) != 2:
    print("Usage: python3 pad-output.py <input_file>")
    sys.exit(1)

input_file = sys.argv[1]
chunk_length = 70

# Call the pad_output function with the specified arguments
pad_output(input_file, chunk_length)