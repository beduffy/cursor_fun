import time
import json
import os
import subprocess

PRESETS_FILE = 'presets.json'

class IntervalTrainingApp:
    def __init__(self):
        self.load_presets()

    def load_presets(self):
        if os.path.exists(PRESETS_FILE):
            with open(PRESETS_FILE, 'r') as file:
                self.presets = json.load(file)
        else:
            self.presets = {}

    def save_presets(self):
        with open(PRESETS_FILE, 'w') as file:
            json.dump(self.presets, file, indent=4)

    def add_preset(self, name, events):
        self.presets[name] = events
        self.save_presets()

    def modify_preset(self, name, events):
        if name in self.presets:
            self.presets[name] = events
            self.save_presets()
        else:
            print(f"Preset '{name}' does not exist.")

    def start_training(self, preset_name):
        if preset_name not in self.presets:
            print(f"Preset '{preset_name}' does not exist.")
            return

        events = self.presets[preset_name]
        for event in events:
            print(f"Starting {event['name']} for {event['duration']} seconds.")
            self.speak(f"Starting {event['name']} for {event['duration']} seconds.")
            self.countdown(event['duration'])
            next_event = self.get_next_event(events, event)
            print(f"Next up: {next_event}")
            if next_event != "Training complete!":
                self.speak(f"Next up: {next_event}")
                self.countdown(3, pre_countdown=True)

    def countdown(self, duration, pre_countdown=False):
        if pre_countdown:
            for i in range(duration, 0, -1):
                print(f"{i}", end='\r')
                self.speak(f"{i}")
                time.sleep(1)
        else:
            while duration:
                mins, secs = divmod(duration, 60)
                timeformat = '{:02d}:{:02d}'.format(mins, secs)
                print(timeformat, end='\r')
                time.sleep(1)
                duration -= 1
            print("00:00")

    def get_next_event(self, events, current_event):
        current_index = events.index(current_event)
        if current_index + 1 < len(events):
            return events[current_index + 1]['name']
        else:
            return "Training complete!"

    def speak(self, text):
        subprocess.run(['espeak', text])

if __name__ == "__main__":
    app = IntervalTrainingApp()
    # app.add_preset("Sample Workout", [
    #     {"name": "Warm-up", "duration": 5},
    #     {"name": "Run", "duration": 120},
    #     {"name": "Walk", "duration": 60},
    #     {"name": "Cool-down", "duration": 60}
    # ])
    # app.add_preset("Sample Workout", [
    #     {"name": "Warm-up",   "duration": 5},
    #     {"name": "Run",       "duration": 120},
    #     {"name": "Walk",      "duration": 60},
    #     {"name": "Cool-down", "duration": 60}
    # ])
    app.add_preset("Breaking 1hr 45min", [
        {"name": "Countdown",   "duration": 10},
        {"name": "Top rocks",       "duration": 5 * 60},
        {"name": "Shoulder",      "duration": 5 * 60},
        {"name": "Footwork",      "duration": 15 * 60},
        {"name": "Powermoves",      "duration": 25 * 60},
        {"name": "Freezes",      "duration": 15 * 60},
        {"name": "Choreo",      "duration": 5 * 60},
        {"name": "Combine/integrate/dance",      "duration": 5 * 60},
        {"name": "Back",      "duration": 10 * 60},
        {"name": "Stretch",      "duration": 10 * 60}
    ])
    

    app.start_training("Sample Workout")

    # TODO countdown starts after the next event is started
    # TODO easy way to do minutes?


