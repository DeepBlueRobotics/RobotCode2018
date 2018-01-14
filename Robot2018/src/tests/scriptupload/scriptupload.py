#nt setup
from networktables import NetworkTables
NetworkTables.initialize(server='roboRIO-199-FRC.local')
prefs = NetworkTables.getTable("Preferences")

go = False #true when file has been successfully read
file = [] #an array of strings which represent each line of the file
filename = input("File name: ") #the name of the file to read (user input)

#loops until a file is read into the file array
while not go:
    try:
        if filename == "quit":
            quit()
            
        with open(filename) as script:
            file = script.readlines()
            #balex's testing
            for line in file:
                print(repr(line))
            
        break
    except:
        filename = input("Not found. Try another name (enter to quit): ") or "quit" #retry, or quit (in case the file doesn't exist)

#puts the string array
prefs.putStringArray("autoscript", file)
print("Uploaded %s as a String[] to key \"autoscript\"" % filename)
