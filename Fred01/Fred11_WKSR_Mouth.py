#########################################################
# Fred11 Voice Recognition and Text To Speech
#########################################################
# We will be using the following services:
#    Runtime Service
#    WebGui Service
#    WebkitSpeechRecognition Service (WKSR)
#    MarySpeech Service Text To Speech (TTS)
#    RemoteAdapter Service
#    execfile(file) Python Command
#########################################################

# Before we start I want to setup a variable for later use
# RunningFolder holds the name of the sub directory that will be holding these scripts
# While only the last line of this series of four line actualy sets the variable
# it does allow me to show how to check is a variable already exists and then set 
# it if it is not found.
try:
    RunningFolder
except NameError:
    RunningFolder="fred/fred01"

# The WKSR service is a web based system, 
# in order to use it we will need to create the WebGui service first
# But we don't want to start it just yet so we just use the create command
# GUI in case you didn't know stands for Graphical User Interface.
# All service are created by the Runtime Service so the command is run from there.
# It returns an object that is the WebGui service.
WebGui = Runtime.create("WebGui","WebGui")

# The next this we need to do is prevent the Web Browser starting up on a page we don't need.
# by default when the WebGui service is started it will launce the local web browser for a runtime GUI
# We will prevent this behaveiour by setting the autoStartBrowser to false.
WebGui.autoStartBrowser(False)

# Now we have the web browser disabled from auto starting lets start the WebGui service
# with the startService command
WebGui.startService()

# Ok now we need to create and start the WebKitSpeechRecognition Service
# As with all service, it is created from the Runtime Service and can be started from there
wksr = Runtime.start("wksr","WebkitSpeechRecognition")

# Now that we have the WebGui service and the WebKitSpeechRecognition Service running we can 
# start the Web Browser sending it to a service page we just created.
# It is very inportant that the part of the path after the service is the same 
# name that you give your WebKitSpeechRecognition service in this case wksr
WebGui.startBrowser("http://localhost:8888/#/service/wksr")

# Now I'm an English speaker in Australia, so i will set the language to "en-AU"
# with the setLanguage command.
# for a full list of supported languages visit: https://cloud.google.com/speech/docs/languages
wksr.setLanguage("en-AU")

# If setAutoListen is True, webkitspeech red microphone will auto rearm. 
# microphone will shutdown too if mouth is activated. 
# Careful if this is set to True : You cannot control anymore red microphone from webgui 
# You need to control it from SwinGui, or usually from code
wksr.setAutoListen(False)
 
# If setContinuous is False, this speeds up recognition processing 
# If setContinuous is True, you have some time to speak again, in case of error
# in this case we will use False
wksr.setContinuous(False)

# So far we have delt only with the speech recognition, but now it's time to start talking.
# For the Text To Speech (TTS) we will be useing the MarySpeech TTS service
# First thing we need to do is create it with the Runtime Service
#mouth = Runtime.start("mouth", "MarySpeech")

# To set a different voice use the following command
# To find a list of different voices you can use visit http://myrobotlab.org/service/MarySpeech
##mouth.setVoice("dfki-spike")

from org.myrobotlab.service import Speech
lang="EN" #for NaturalReaderSpeech
Voice="Ryan" 
voiceType = Voice
speech = Runtime.createAndStart("Speech", "NaturalReaderSpeech")
speech.setVoice(voiceType)
speech.setLanguage(lang)

#mouth = Runtime.createAndStart("Mouth","MouthControl")
#sleep(20) # fix for servo attach timing issue as at myrobotlab 236x development builds

# There is nothing worse that your robot talking to its self
# to prevent that from happening we will add the mouth service to the wksr service
wksr.addMouth(speech)

# At this point the startup process will have taken quite a while so lets let everyone we are
# Progressing through the startup ok.
speech.speakBlocking("Hello world")
speech.speakBlocking("Please wait while I start up my motor cortex.")


