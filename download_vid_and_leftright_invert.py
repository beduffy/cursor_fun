import yt_dlp
import cv2
import numpy as np
import os


def download_and_invert_video(url, output_filename):
    # Download the video
    ydl_opts = {
        'outtmpl': 'temp_video.%(ext)s',
        'format': 'bestvideo[ext=mp4]+bestaudio[ext=m4a]/best[ext=mp4]/best',
    }
    
    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        ydl.download([url])
    
    # Open the downloaded video
    cap = cv2.VideoCapture('temp_video.mp4')
    
    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))
    
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame_count += 1
            # Flip the frame horizontally
            flipped_frame = cv2.flip(frame, 1)
            
            # Write the flipped frame
            out.write(flipped_frame)

            # Print frame count out of total num frames every 50 frames
            if frame_count % 50 == 0:
                print(f"Processing frame {frame_count} out of {total_frames}")
        else:
            break
    
    # Release everything
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    # Remove the temporary downloaded file
    os.remove('temp_video.mp4')
    
    print(f"Video processed and saved as {output_filename}")


# Example usage
# youtube_url = "https://www.youtube.com/watch?v=dQw4w9WgXcQ"  # Replace with your desired YouTube URL
youtube_url = "https://www.youtube.com/watch?v=4inTAnc4GbI"  # Replace with your desired YouTube URL
output_file = "inverted_video.mp4"
download_and_invert_video(youtube_url, output_file)
