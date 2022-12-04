#------------------------------------
#Define Variables
#------------------------------------
processComments=False

#------------------------------------
#Define Functions
#------------------------------------

def ID_KRL(codeline):
    print("Code: " + codeline)

#------------------------------------
# Main Code
#------------------------------------
# Open File
path=r"E:\Desktop\Daniel\Programming\GitRepository\Scara-Master\scara-mc-master\Interpreters\Kuka\UP_Service.src"
path=path.replace("\\","/")
print("Opening File:" + path)
file = open(path)

#line Number counter
lineNr=0
#go trough file line by line
for line in file:
    lineNr+=1
    #skip empty lines
    if line.isspace()==False:
        #extract comments by splitting at the first ";" symbol
        text = line.split(";",1)
        #print line (if not all spaces)
        text[0]+=" " #add a space to a potentially empty string to check empty and zero strings
        if text[0].isspace()==False:
            #print line number
            print("Line: " + str(lineNr))
            #Analze Code
            ID_KRL(text[0].strip())
        #print comment if exists
        if len(text)!=1 and processComments:
            #line contained a comment      
            print("Comment: " + text[1].strip())
#close file
file.close()