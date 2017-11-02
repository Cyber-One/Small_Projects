#########################################################
# Fred10 Slave Controller Setup
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

# This file is being used as the master setup for this Raspberry Pi 3
# in order the help keep the various files smaller and therefore easier to find section of code
# we need a way of calling code to be executed from other files
# Enter the execfile command.
execfile(RunningFolder+'/Fred12_RPi_Controller.py')
execfile(RunningFolder+'/Fred11_WKSR_Mouth.py')

# Now that we have somewhere for our servos to attach to
# lets create some servo functions
# We will Start with the Head
execfile(RunningFolder+'/Fred13_HeadServos.py')
execfile(RunningFolder+'/Fred14_RightArmServos.py')
execfile(RunningFolder+'/Fred15_LeftArmServos.py')
execfile(RunningFolder+'/Fred16_BackServos.py')

fred = Runtime.createAndStart("fred", "ProgramAB")

# Next we need to start a session. The first parameter here is your name and is used to store data
# created as a result of talking to you.
# The second parameter is the name of the aiml file set that ProgramAB will be using.
fred.startSession("ray", "alice2")

# The information coming out of ProgramAB can have some Hyper Text Markup Language (HTML) included in it.
# MarySpeech does not like the markup, so we need to get rid of it.
# To do that we use a HtmlFilter Service
# From Runtime, we create the HtmlFilter Service
htmlfilter = Runtime.createAndStart("htmlfilter", "HtmlFilter")

# Next we will add what is kown as data routing.
# We will route text created by fred (ProgramAB) to the HtmlFilter
fred.addTextListener(htmlfilter)

# Next we will tell WKSR from Fred01 to send the text it has converted from speech to fred the ProgramAB service
wksr.addTextListener(fred)
#python.subscribe("fred", "wksr", "addTextListener", "fred")

# And Finally, the filtered text from the HtmlFilter to the MarySpeech of Fred01 we called mouth
htmlfilter.addTextListener(speech)
#htmlfilter.subscribe("fred", "speech")

def onEndSpeaking(text):
    sleep(.5)   
    #Start of main script
    sleep(1)
    speech.speakBlocking(text)
    jawServo.rest()
#    mouth.jaw.moveTo(110)
    


def saystuff():
    myvoices = ['Ryan']
    myvoicescount = len(myvoices)
    for i in range(0,myvoicescount):
        speech.setVoice(myvoices[i])
        onEndSpeaking ("I'm completely operational, and all my circuits are functioning perfectly.")
        

saystuff()

wksr.setAutoListen(True)
wksr.startListening()