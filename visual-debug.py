import os
import sys
import readline
import tty
import termios


NEGEDGE_LINE_ST = "-----------------------------------------NEGATIVE EDGE OF CLOCK CYCLE"
POSEDGE_LINE_ST = "+++++++++++++++++++++++++++++++++++++++++++++POSITIVE EDGE OF CLOCK CYCLE"

NEGEDGE_ST_IDX = 17


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

            if num_pad_lines > 0:
                for i in range(num_pad_lines):
                    pad_line = "[" + str(line_no) + "] \n"
                    file.write(pad_line)
                    line_no += 1


def getch():
    import sys, termios, tty

    fd = sys.stdin.fileno()
    orig = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)  # or tty.setraw(fd) if you prefer raw mode's behavior.
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, orig)

def interactive_session(input_file):
    # Open the input file in read mode
    with open(input_file, 'r') as file:
        # Read all lines from the input file and store them in a list
        lines = file.readlines()

    chunks = []  # List to store chunks of lines
    chunk = []  # List to store lines in a chunk
    for line in lines:
        if line.startswith(POSEDGE_LINE_ST):
            # If a line starts with '++++', it indicates the start of a new chunk
            if chunk:
                # If there is a non-empty chunk, append it to the list of chunks
                chunks.append(chunk)
                chunk = []  # Reset the chunk list
        chunk.append(line)  # Add the line to the current chunk

    if chunk:
        # If there is a non-empty chunk remaining, append it to the list of chunks
        chunks.append(chunk)
    
    current_chunk_index = 0  # Index of the current chunk
    while True:
        # Print the current chunk
        os.system('clear')
        line_no = 0
        for line in chunks[current_chunk_index]:
            if (line.startswith(NEGEDGE_LINE_ST)):
                if (NEGEDGE_ST_IDX - line_no) > 0:
                    print("\n" * (NEGEDGE_ST_IDX- line_no))

            print(line, end='')
            line_no += 1

        # Prompt the user for input
       # user_input = input("Press 'n' for next chunk, 'b' for previous chunk, or 'q' to quit: ")
        user_input = getch()
        if user_input == 'n':
            # Move to the next chunk if available
            if current_chunk_index < len(chunks) - 1:
                current_chunk_index += 1
            else:
                print("No more chunks available.")
        elif user_input == 'b':
            # Move to the previous chunk if available
            if current_chunk_index > 0:
                current_chunk_index -= 1
            else:
                print("Already at the first chunk.")
        elif user_input == 'q':
            # Quit the interactive session
            break
        else:
            print("Invalid input. Please try again.")


# Usage example
if len(sys.argv) != 2:
    print("Usage: python3 pad-output.py <input_file>")
    sys.exit(1)

input_file = sys.argv[1]
chunk_length = 70

# Call the pad_output function with the specified arguments
pad_output(input_file, chunk_length)

# Call the interactive_session function with the chunks
interactive_session(input_file)