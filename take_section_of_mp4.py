import moviepy.editor as mp

def trim_video(video_path, start_sec, end_sec):
    # Load the video file
    video = mp.VideoFileClip(video_path)
    
    # Trim the video
    trimmed_video = video.subclip(start_sec, end_sec)
    
    # Create new video path
    base_name = video_path.rsplit('.', 1)[0]
    extension = video_path.rsplit('.', 1)[1]
    new_video_path = f"{base_name}_section_{start_sec}_{end_sec}.{extension}"
    
    # Write the trimmed video to a new file
    trimmed_video.write_videofile(new_video_path, codec='libx264')
    
    print(f"Trimmed video saved to {new_video_path}")

# Example usage
if __name__ == "__main__":
    # video_path = "/home/ben/Videos/Webcam/mp4_conversions/2024-06-04-130416.mp4"
    # video_path = "/home/ben/Videos/Webcam/mp4_conversions/2024-06-16-121057.mp4"
    # video_path = "/home/ben/Videos/Webcam/mp4_conversions/2024-06-19-203212.mp4"
    video_path = "/home/ben/Videos/Webcam/mp4_conversions/2024-08-18-180025.mp4"
    start_sec = 73
    end_sec = 125
    trim_video(video_path, start_sec, end_sec)
