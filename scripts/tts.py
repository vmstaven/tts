#!/usr/bin/env python

import sys
import pyttsx3

# Initiate tts engine.
engine = pyttsx3.init()

# Extract the string.
ttsStr = sys.argv

del ttsStr[0]

# Say each word independent.
for word in range(len(ttsStr)):
    engine.say(ttsStr[word])

engine.runAndWait()
