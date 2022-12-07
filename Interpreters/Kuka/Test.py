
import KRL

#------------------------------------
# Define Variables
#------------------------------------
bShowComments=False
path=r"E:\Desktop\Daniel\Programming\GitRepository\Scara-Master\scara-mc-master\Interpreters\Kuka\UP_Service.src"
robProgPath=r"E:\Desktop\Daniel\Programming\GitRepository\Scara-Master\scara-mc-master\Interpreters\Kuka\tempProg.py"

dicOperators={
    #dictionary is weighted, so specialised command words should be at the top of the keylist (eg "default" above "def")
    "TRIGGER WHEN":"ID_triggerWhen", #trigger distance to point
    "$BWDSTART":"ID_setBwdstart",
    "PDAT_ACT":"ID_setPdatAct",
    "FDAT_ACT":"ID_setFdatAct",
    "BAS":"ID_setBase",
    "SET_CD_PARAMS":"ID_setCdParams",
    "PTP":"ID_movePtp",
    "WHILE":"ID_whileOp",
    "SWITCH":"ID_switchOp",
    "CASE":"ID_caseOp",
    "DEFAULT":"ID_defaultOp",
    "LOOP":"ID_loopOp",
    "HALT":"ID_haltOp",
    "ENDLOOP":"ID_endloopOp",
    "ENDSWITCH":"ID_endswitchOp",
    "ENDWHILE":"ID_endwhileOp",
    "END":"ID_funcEnd",
    "GLOBAL DEF":"ID_funcStartGlobal",
    "DEF":"ID_funcStart"
}

keyOperators=dicOperators.keys()
indent="" #indentation in output file

#------------------------------------
# Define Functions
#------------------------------------

#Ids KRL function from library
def ID_KRL(code,file):
    global indent
    #print("Code: " + code)
    #split codeblock into operators,parameters and variables
    for key in keyOperators:
        if code.startswith(key):
            #Get keywords and varables by removing the operator from "code" string
            params=code[len(key):].strip()
            print("Command: " + key +" Parameters: " + params)

            #call function from KRL library
            krlFunc = getattr(KRL, dicOperators[key])
            output=krlFunc(params)
            #write in temp file
            if output[1]==10 and indent!="": indent=indent[:-1] #unindent and the re indent
            elif output[1]==-1 and indent!="": indent=indent[:-1]
            file.write( indent + output[0]+"\n")

            #handle indenting
            if output[1]==1 or output[1]==10: indent=indent+" "
            
            break
    else: #if for loop was not exited with break function
        file.write( indent + "#"+ code +"\n")

#------------------------------------
# Main Code
#------------------------------------
#open File
path=path.replace("\\","/")
print("Opening File:" + path)
file = open(path)

robProgPath=robProgPath.replace("\\","/")
print("Robot Programm:" + robProgPath)
#open file overwriting content
robprog=open(robProgPath,"w")
robprog.write("#Temp PY Robot Programm\n")
robprog.write("import KRL\n")

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
            ID_KRL(line[0].strip(),robprog)

        #display comment (optional)
        if len(line)!=1 and bShowComments:      
            print("Comment: " + line[1].strip())

#close file
file.close()

robprog.close()