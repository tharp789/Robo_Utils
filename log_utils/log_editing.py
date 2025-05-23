def load_txt_file(file_path):
    """
    Load a text file and return its content as a list of lines.
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()
    return lines

def save_txt_file(file_path, lines):
    """
    Save a list of lines to a text file.
    """
    with open(file_path, 'w') as file:
        file.writelines(lines)

def remove_lines_by_keywords(lines, keywords):
    """
    Remove lines from a list that contain any of the specified keywords.
    """
    filtered_lines = []
    for line in lines:
        if not any(keyword in line for keyword in keywords):
            filtered_lines.append(line)
    return filtered_lines

if __name__ == "__main__":
    # Example usage
    input_file_path = "/home/tyler/Downloads/16-58-rgb-stream-not-playable.log"
    output_file_path = "/home/tyler/Downloads/filtered_log.txt"
    keywords_to_remove = ["foxglove_bridge", "mavros_node", "doodle_companion_publisher"]

    # Load the file
    lines = load_txt_file(input_file_path)

    # Remove lines containing the specified keywords
    filtered_lines = remove_lines_by_keywords(lines, keywords_to_remove)

    # Save the filtered lines to a new file
    save_txt_file(output_file_path, filtered_lines)
