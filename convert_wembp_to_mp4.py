import os
import subprocess

def convert_webm_to_mp4(input_folder, output_folder):
    """
    Convert all .webm files in the input_folder to .mp4 files in the output_folder.
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in os.listdir(input_folder):
        if filename.endswith(".webm"):
            input_path = os.path.join(input_folder, filename)
            output_filename = os.path.splitext(filename)[0] + ".mp4"
            output_path = os.path.join(output_folder, output_filename)
            
            command = [
                "ffmpeg",
                "-i", input_path,
                "-y",  # Overwrite
                "-b:v", "1M",  # Set video bitrate to 1 Megabit per second
                "-b:a", "128k",  # Set audio bitrate to 128 Kilobits per second
                output_path
            ]
            
            subprocess.run(command, check=True)
            print(f"Converted {input_path} to {output_path}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert .webm files to .mp4 format.")
    parser.add_argument("input_folder", type=str, help="Path to the folder containing .webm files.")
    parser.add_argument("output_folder", type=str, help="Path to the folder to save .mp4 files.")

    args = parser.parse_args()

    convert_webm_to_mp4(args.input_folder, args.output_folder)
