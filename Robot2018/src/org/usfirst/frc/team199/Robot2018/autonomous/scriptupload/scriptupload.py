#nt setup
from networktables import NetworkTables
import __future__
NetworkTables.initialize(server='10.1.99.2')
prefs = NetworkTables.getTable("Preferences")

go = False #true when file has been successfully read
lines = [] #an array of strings which represent each line of the file
filename = raw_input("File name: ") #the name of the file to read (user input)
oneline = "" #the file as a single string
retry = False #whether or not to retry.

if not NetworkTables.isConnected():
    print("You aren't connected to the bot.")
    quit()
#loops until a file is read into the file array
while not go:
    if retry:
        filename = raw_input("Not found. Try another name (enter to quit): ")#retry, or quit (in case the file doesn't exist)
        if filename == "":
            quit()
    try:
        with open(filename) as script:
            lines = script.readlines()
            oneline = "".join(lines) #this is python's weird syntax for joining an array into one line, as we can't seem to pull string arrays from the WPILib Preferences class.
        break
    except:
        retry = True
#puts the string
prefs.putString("autoscripts", oneline)
print("Uploading %s as a String[] to key \"autoscripts\"" % filename)

#checks if the key has been filled
tester = "" #a variable to check if autoscripts is None

tester = prefs.getValue("autoscripts", None)
if tester != oneline:
    print("It doesn't look like key \"autoscripts\" is filled, maybe you aren't connected to networktables?")
else:
    print("Success!")
