
import KRL

#------------------------------------
# Define Variables
#------------------------------------
bShowComments=False
path=r"E:\Desktop\Daniel\Programming\GitRepository\Scara-Master\scara-mc-master\Interpreters\Kuka\UP_Service.src"
dicOperators={
    #dictionary is weighted, so specialised command words should be at the top of the keylist (eg "default" above "def")
    "TRIGGER WHEN DISTANCE":"triggerWhenDistance", #trigger distance to point
    "$BWDSTART":"setBwdstart",
    "PDAT_ACT":"setPdatAct",
    "FDAT_ACT":"setFdatAct",
    "BAS":"setBase",
    "SET_CD_PARAMS":"setCdParams",
    "PTP":"movePtp",
    "CONTINUE":"continueOp",
    "WHILE":"whileOp",
    "SWITCH":"switchOp",
    "CASE":"caseOp",
    "DEFAULT":"defaultOp",
    "LOOP":"loopOp",
    "HALT":"halt",
    "ENDLOOP":"endloopOp",
    "ENDSWITCH":"endswitchOp",
    "ENDWHILE":"endwhileOp",
    "END":"funcEnd",
    "GLOBAL DEF":"funcStartGlobal",
    "DEF":"funcStart"
}
keyOperators=dicOperators.keys()
#------------------------------------
# Define Functions
#------------------------------------

def ID_KRL(code):
    #print("Code: " + code)
    #split codeblock into operators,parameters and variables
    for key in keyOperators:
        if code.startswith(key):
            print("Command: " + key)

            #Get keywords and varables by removing the operator from "code" string
            params=code[len(key):].strip()
            print("Parameters: " + params)

            #call function from KRL library
            krlFunc = getattr(KRL, dicOperators[key])
            krlFunc()
            break
    else: #if for loop was not exited with break function
        print("Unknown command: " + code)
        
#------------------------------------
# Main Code
#------------------------------------
#open File
path=path.replace("\\","/")
print("Opening File:" + path)
file = open(path)

#go trough file line by line
for numLine, line in enumerate(file):

    #skip empty lines
    if line.isspace()==False:
    
        #extract comments by splitting at the first ";" symbol
        line = line.split(";",1)

        #check for lines with no code
        line[0]+=" " #add a space to a potentially empty string to check empty and zero strings
        if line[0].isspace()==False:
            #print line number
            print("Line: " + str(numLine))
            #Analze Code
            ID_KRL(line[0].strip())

        #display comment (optional)
        if len(line)!=1 and bShowComments:      
            print("Comment: " + line[1].strip())

#close file
file.close()