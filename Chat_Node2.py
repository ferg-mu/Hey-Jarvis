import struct
import pyaudio
import pvporcupine
import speech_recognition as sr
import pyttsx3
import paho.mqtt.client as paho
import openai
import time

# MQTT Topics
TOPIC_CONTROL = "/robot_control"
TOPIC_BRICK = "/brick"
TOPIC_SPEAK_POS = "/speak_pos"
TOPIC_SPEAK_SPEECH = "/speech"
TOPIC_SPEAK = "/speak"

start = True

# Global variables
last_message = "None"
previous_message = "None"
topic = "None"
brick = 0
previous_brick = 3

# Access key for Porcupine
access_key = 

# Initialize recognizer and text-to-speech engine
recognizer = sr.Recognizer()
engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.9)
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[1].id)  # Select a voice

# OpenAI API key
openai.api_key = "sk-proj-ozvMcP8VssnpzBzOp1vzT3BlbkFJYO0JgX5uyhpo3dhNK6FJ"

# Porcupine wake word parameters
keyword = "jarvis"
porcupine = None
paud = None
audio_stream = None

# MQTT callbacks
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"CONNACK received with code {rc}.")

def on_publish(client, userdata, mid, properties=None):
    print(f"mid: {mid}")

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print(f"Subscribed: {mid} {granted_qos}")

def on_message(client, userdata, msg):
    global last_message, topic, speech
    new_message = msg.payload.decode('utf-8')
    topic = msg.topic
    if new_message != last_message and topic == TOPIC_SPEAK:
        last_message = msg.payload.decode('utf-8')
        print("speak:" + last_message)
    elif topic == TOPIC_SPEAK_SPEECH:
        speech = msg.payload.decode('utf-8')
        print("speech:" + speech)

# Listen and recognize speech
def listen_and_recognize():
    global start
    with sr.Microphone() as source:
        if start:
            speak_text("Hey, are you ready to collaborate on building the tower?")
            recognizer.adjust_for_ambient_noise(source)
            print("Listening for command...")
        else:
            recognizer.adjust_for_ambient_noise(source)
            print("Listening for command...")
            speak_text("Hey")
        audio = recognizer.listen(source)
        try:
            print("Recognizing...")
            text = recognizer.recognize_google(audio)
            print(f"Recognized: {text}")
            return text
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
            return None

# Text-to-speech function
def speak_text(text):
    engine.say(text)
    engine.runAndWait()

# Wake word detected callback
def detected_callback():
    print("Wake word detected!")
    recognized_text = listen_and_recognize()
    if recognized_text:
        process_recognized_text(recognized_text)

# Process recognized text and interact with GPT-3.5
def process_recognized_text(recognized_text):
    global start, brick

    predefined_responses = {
        "yes": "Nice! I will place the first stick",
        "no": "Let me know when you are ready then! Just say: Hey Jarvis.",
        "maybe": "Let me know when you are ready then! Just say: Hey Jarvis.",
        "stop": "Stopping the robot.",
        "pause": "Pausing the robot.",
        "play": "Resuming the robot.",
        "resume": "Resuming the robot.",
        "continue": "Continuing the robot."
    }

    # Check for predefined responses
    for key, response in predefined_responses.items():
        if key in recognized_text.lower():
            speak_text(response)
            if key in ["yes", "play", "resume", "continue"]:
                client.publish(TOPIC_BRICK, payload=brick, qos=0)
                start = False
            elif key in ["stop", "pause"]:
                client.publish(TOPIC_CONTROL, payload=key.upper(), qos=0)
            return

    # If no predefined response, use GPT-3.5
    try:
        messages = [
            {"role": "system", "content": "You are a helpful assistant and playing the part of the UR Robot. You are collaborating on building a tower with a human."},
            {"role": "system", "content": "You only communicate about topic related to the task at hand"},
            {"role": "system", "content": "For context. You are a UR-10 Robotic Arm. You have multiple rows of stacked bricks infront of you. Infront of you is a human, the human also has multiple bricks laying infront of them. Inbetween you two is a taped square, within you are building the a tower of of the wooden sticks. You are starting and placing the first one and the human is doing the next until the tower is fully built."},
            {"role": "system", "content": "Each Brick is around 75mm long"},
            {"role": "system", "content": "You are using a Vision System with a webcam to track the placement and inform the user if it is placed correctly or not. If not you are prompting the human on possilbe problems"},
            {"role": "system", "content": "Possible issues could be, the brick is outside the taped lines it should be in; The brick is not straight; the brick is not placed in the correct position within the taped lines"},
            {"role": "system", "content": "Each Brick has a green line centerline for identification with the robot"},
            {"role": "system", "content": "The Goal is to build a tall, stable tower and make it accurate to reduce the need to restart or rebuild"},
            {"role": "system", "content": "The Goal is to build a tall, stable tower and make it accurate to reduce the need to restart or rebuild"},
            {"role": "user", "content": recognized_text}
        ]
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=messages,
            max_tokens=150
        )
        response_text = response['choices'][0]['message']['content'].strip()
        speak_text(response_text)
    except Exception as e:
        print(f"An error occurred while trying to generate a response: {e}")

# Main function
try:
    # Initialize MQTT client
    client = paho.Client(client_id="", protocol=paho.MQTTv5)
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    client.connect("mqtt-dashboard.com", 1883)

    # Initialize Porcupine and PyAudio
    porcupine = pvporcupine.create(access_key=access_key, keywords=[keyword])
    paud = pyaudio.PyAudio()
    audio_stream = paud.open(rate=porcupine.sample_rate, channels=1, format=pyaudio.paInt16, input=True, frames_per_buffer=porcupine.frame_length)

    # Subscribe to MQTT topic and start the loop
    client.subscribe(TOPIC_SPEAK, qos=1)
    client.subscribe(TOPIC_SPEAK_SPEECH, qos=1)
    client.loop_start()

    while True:
        if last_message == "True" and topic == TOPIC_SPEAK and brick != previous_brick:
            speak_text(speech)
            last_message = "False"
            if ";" in speech.lower():
                previous_brick = brick
                brick += 1
                if brick > 3:
                    brick = 0
                client.publish(TOPIC_BRICK, payload=str(brick), qos=0)
                client.publish(TOPIC_SPEAK_POS, payload="False", qos=1)
            client.publish(TOPIC_SPEAK, payload="False", qos=1)
            time.sleep(3.0)

        audio_frame = audio_stream.read(porcupine.frame_length)
        audio_data = struct.unpack_from("h" * porcupine.frame_length, audio_frame)
        keyword_index = porcupine.process(audio_data)

        if keyword_index >= 0:
            detected_callback()

finally:
    if porcupine is not None:
        porcupine.delete()
    if audio_stream is not None:
        audio_stream.close()
    if paud is not None:
        paud.terminate()
