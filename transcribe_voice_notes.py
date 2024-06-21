# import speech_recognition as sr

# def transcribe_voice_to_file(output_file):
#     recognizer = sr.Recognizer()
#     microphone = sr.Microphone()

#     with microphone as source:
#         print("Adjusting for ambient noise, please wait...")
#         recognizer.adjust_for_ambient_noise(source)
#         print("Listening for your voice...")
#         audio = recognizer.listen(source, timeout=50, phrase_time_limit=60)  # Record for longer

#     try:
#         print("Transcribing audio...")
#         transcription = recognizer.recognize_google(audio)
#         with open(output_file, 'w') as file:
#             file.write(transcription)
#         print(f"Transcription saved to {output_file}")
#     except sr.UnknownValueError:
#         print("Google Speech Recognition could not understand the audio")
#     except sr.RequestError as e:
#         print(f"Could not request results from Google Speech Recognition service; {e}")

# if __name__ == "__main__":
#     output_file = 'transcription.txt'
#     transcribe_voice_to_file(output_file)

#


import speech_recognition as sr

def transcribe_voice_to_file(output_file):
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    with open(output_file, 'w') as file:
        while True:
            with microphone as source:
                print("Adjusting for ambient noise, please wait...")
                recognizer.adjust_for_ambient_noise(source)
                print("Listening for your voice...")
                audio = recognizer.listen(source, timeout=50, phrase_time_limit=60)  # Record for longer

            try:
                print("Transcribing audio...")
                transcription = recognizer.recognize_google(audio)
                print(f"Transcription: {transcription}")
                file.write(transcription + "\n")
                
                if "stop recording" in transcription.lower():
                    print("Stop command detected. Ending transcription.")
                    break
            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand the audio")
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service; {e}")

if __name__ == "__main__":
    output_file = 'transcription.txt'
    transcribe_voice_to_file(output_file)