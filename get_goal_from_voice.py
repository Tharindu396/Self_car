from vosk import Model, KaldiRecognizer
import pyaudio, json, os

MODEL_PATH = os.path.join(os.getcwd(), "model")
model = Model(MODEL_PATH)
recognizer = KaldiRecognizer(model, 16000)

LETTER_MAP = {
    "a": "A", "eh": "A", "hey": "A",
    "b": "B", "bee": "B", "be": "B",
    "c": "C", "see": "C", "sea": "C",
    "d": "D","the": "D", "he": "D",
    "e": "E", "ee": "E",
    "if": "F", "ef": "F",
    "g": "G", "gee": "G",
    "it": "H", "aitch": "H",
    "i": "I", "eye": "I","are you": "I",
}

def normalize_letter(text):
    text = text.strip().lower()
    return LETTER_MAP.get(text)

def get_goal_from_voice():
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
    stream.start_stream()

    print("Say destination letter...")

    while True:
        data = stream.read(4000, exception_on_overflow=False)
        if recognizer.AcceptWaveform(data):
            text = json.loads(recognizer.Result())["text"]
            if text:
                return text.lower()

if __name__ == "__main__":
    print("Listening for destination...")
    heard = get_goal_from_voice()
    clean_letter = normalize_letter(heard)

    if clean_letter:
        print(f"Detected node letter: {clean_letter}")
        print(f"Going to {clean_letter}.")
    else:
        print(f"Unknown input: '{heard}' — try again.")
