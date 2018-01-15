#nt setup
from networktables import NetworkTables
import __future__
NetworkTables.initialize(server='roboRIO-199-FRC.local')
prefs = NetworkTables.getTable("Preferences")

go = False #true when file has been successfully read
lines = [] #an array of strings which represent each line of the file
filename = raw_input("File name: ") #the name of the file to read (user input)
oneline = "" #the file as a single string
retry = False

#loops until a file is read into the file array
while not go:
    if retry:
        filename = raw_input("Not found. Try another name (enter to quit): ")#retry, or quit (in case the file doesn't exist)
        if filename == "":
            quit()
    try:
        with open(filename) as script:
            lines = script.readlines()
            "".join(lines)
        break
    except:
        retry = True
#puts the string array
prefs.putString("autoscripts", file)
prefs.putStringArray("autoscripts", file)
print("Uploaded %s as a String[] to key \"autoscripts\"" % filename)