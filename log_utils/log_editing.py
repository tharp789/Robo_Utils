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
        if not any(keyword in line.lower() for keyword in keywords):
            filtered_lines.append(line)
    return filtered_lines

def keep_lines_by_keywords(lines, keywords):
    """
    Keep only lines from a list that contain any of the specified keywords.
    """
    kept_lines = []
    for line in lines:
        if any(keyword in line.lower() for keyword in keywords):
            kept_lines.append(line)
    return kept_lines

if __name__ == "__main__":
    # Example usage
    input_file_path = "/home/tyler/Downloads/docker.log"
    output_file_path = "/home/tyler/Downloads/filtered_log.txt"

    # Load the file
    lines = load_txt_file(input_file_path)

    # Remove lines containing the specified keywords
    keywords_to_remove = ["foxglove_bridge", "mavros_node", "doodle_companion_publisher", "transport", "zenoh"]
    filtered_lines = remove_lines_by_keywords(lines, keywords_to_remove)

    # Keep lines containing the specified keywords
    # keywords_to_keep = ["rtsp_detection", "doodle_companion_publisher"]
    # filtered_lines = keep_lines_by_keywords(lines, keywords_to_keep)

    # Save the filtered lines to a new file
    save_txt_file(output_file_path, filtered_lines)
