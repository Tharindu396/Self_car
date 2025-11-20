from vosk import Model, KaldiRecognizer
import pyaudio, json

model = Model("model")
recognizer = KaldiRecognizer(model, 16000)

def get_goal_from_voice():
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
    stream.start_stream()

    print("Say destination...")

    while True:
        data = stream.read(4000, exception_on_overflow=False)
        if recognizer.AcceptWaveform(data):
            text = json.loads(recognizer.Result())["text"]
            if text:
                return text.lower()

if __name__ == "__main__":
    print("Detecting position...")
    frame = get_camera_frame()
    start = detect_node_id(frame)

    if start is None:
        print("Could not detect position!")
        exit()

    print("Listening for destination...")
    voice_text = get_goal_from_voice()
    goal = goal_map.get(voice_text)

    if goal is None:
        print("Unknown destination command!")
        exit()

    path, distance = dijkstra(graph, start, goal)
    print("Shortest Path:", path)
    print("Distance:", distance)

    follow_path(path)
