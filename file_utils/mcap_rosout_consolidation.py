import sys
import os
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory as Ros2DecoderFactory

LEVEL_MAP = {
    10: "DEBUG",
    20: "INFO",
    30: "WARN",
    40: "ERROR",
    50: "FATAL",
}

def extract_rosout_logs(mcap_file_path, output_txt_path):
    if not os.path.isfile(mcap_file_path):
        print(f"Error: MCAP file '{mcap_file_path}' not found.")
        return

    with open(mcap_file_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[Ros2DecoderFactory()])

        with open(output_txt_path, "w") as out_file:
            for msg_view in reader.iter_decoded_messages():
                if msg_view.channel.topic == "/rosout":
                    msg = msg_view.decoded_message

                    stamp = msg.stamp
                    timestamp = f"{stamp.sec}.{str(stamp.nanosec).zfill(9)}"

                    level = LEVEL_MAP.get(msg.level, f"LVL{msg.level}")
                    name = msg.name
                    text = msg.msg

                    log_line = f"[{timestamp}] [{level}] {name}: {text}\n"
                    out_file.write(log_line)

    print(f"Finished writing logs to {output_txt_path}")

if __name__ == "__main__":

    mcap_file_path = "/home/tyler/Documents/Robo_Utils/file_utils/rosbag_real_2025-04-24-23-29-53_0.mcap"
    output_txt_path = "/home/tyler/Documents/Robo_Utils/file_utils/out.txt"
    extract_rosout_logs(mcap_file_path, output_txt_path)
