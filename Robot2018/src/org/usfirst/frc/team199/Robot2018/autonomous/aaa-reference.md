# Synopsis

AAA is a simple, pseudo-assembly, interpreted language. 

Individual path scripts start with a name + a colon and end with the next script,the end command, or the end of the file. Each command occupies one line, and anything on a line past a pound sign is ignored.

The language should be able to be converted into either PID + turns or motion profiling commands.

Distance arguments are in inches, rotation in arc degrees. All numbers can have a floating point.

Ex:
```
1 | LLRx:
2 | wait 5  # wait for other robots to move past
3 | move (12,24) (30,48.5) 125.75 # move to those points and turn 
4 | scale # place cube in scale
5 | end # end the script (not necessary)
```

# Script Naming Convention

The 4 characters represent 4 values. All paths include crossing the baseline at some point.

1. Robot's start position. 
    * R = right, C = center, L = left.
2. Switch information. 
    * R = going for the right plate, L = going for the left plate, x = not going for switch
3. Scale information. 
    * R = going for the right plate, L = going for the left plate, x = not going for scale
4. Exchange. 
    * E = going for the exchange, x = not going for exchange

Ex:
  
**LLRx**: starting on the left, go for the switch's left plate and then the scale's right plate. don't go for the exchange.  
**Cxxx**: starting in the center, just cross the baseline. don't go for the exchange.  
**LxRx**: starting on the left, go for the scale's right plate. don't go for the exchange.  
**RLxE**: starting on the right, go for the switch on the left, and then go for the exchange.  


# Instruction Reference

<!-- Sorry about how ugly the table is, it's either this or gigantic spacing that also looks ugly if you have word wrap - @kevinzwang -->

| Name | Description | Example Usage |
| --- | --- | ---|
| end | End the script. Anything after will be ignored. | `end` |
| exchange | Feeds a cube at exchange height. | `exchange` |
| intake | Intakes a cube in front of the robot. | `intake` |
| jump | Jumps to the specified script and continues the current script when finished. (Doesn’t make the robot go up.) | `jump MoveToRScale`
| move | Move forward or backwards for the specified amount in inches, relative to the current position. | `move 24` |
| moveto | Move to 1 or more points, sequentially, with an optional last value having a final angle to face towards, all relative to the starting position. | `moveto (12,0) (36,12)` <br> `moveto (0,12) 90` |
| scale | Place a cube at scale height after moving forward by the specified amount with the lift up. | `scale 24` |
| switch | Place a cube at switch height after moving forward by the specified amount with the lift up. | `switch 24` |
| turn | Turn towards a relative point or rotate clockwise by an angle in degrees, relative to the current position. <br> Negative angle for counterclockwise. | `turn (36,48)` <br> `turn 45` |
| wait | Waits for the number of seconds before proceeding to next command. | `wait 5` |
