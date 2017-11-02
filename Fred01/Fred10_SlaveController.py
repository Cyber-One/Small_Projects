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
execfile(RunningFolder+'/Fred11_WKSR_Mouth.py')
execfile(RunningFolder+'/Fred12_RPi_Controller.py')

# Now that we have somewhere for our servos to attach to
# lets create some servo functions
# We will Start with the Head
execfile(RunningFolder+'/Fred13_HeadServos.py')
execfile(RunningFolder+'/Fred14_RightArmServos.py')
execfile(RunningFolder+'/Fred15_LeftArmServos.py')
execfile(RunningFolder+'/Fred16_BackServos.py')

def onEndSpeaking(text):
    sleep(.5)   
    #Start of main script
    sleep(1)
    speech.speakBlocking(text)  
    mouth.jaw.moveTo(110)
    

def saystuff():
    myvoices = ['Ryan']
    myvoicescount = len(myvoices)
    for i in range(0,myvoicescount):
        speech.setVoice(myvoices[i])
        onEndSpeaking ("I'm completely operational, and all my circuits are functioning perfectly.")
        

saystuff()

# now lets start the RemoteAdapter Service. 
# As this will be headless setup that is we will not have a moitor or keyboard, we will listen.
remote = Runtime.createAndStart("remote","RemoteAdapter")
remote.startListening()
