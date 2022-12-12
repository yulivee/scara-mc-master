#-------------------------------------------
# Function and Variables Definitions
#-------------------------------------------

#non zero if programm is in a switch case
firstInSwitch=True
#library for gathering parser keys and values
parserLib={}

def getValueEquals(code):
    A=code.split("=",1) #A1 = Value
    return A[1].strip()

#Function: General move Function
def move(type,posdat,pdat,fdat,cdp,flyby):
    pass

#Function: Prepare Move Ret String
def getMoveParams(type,code):
    A=code.split(" ",1) #A0 = Value, #A1=flyby (optional)
    if len(A)==1:
        B=True
    else:
        B=False
    retString="(\"{}\",{},thisPDAT,thisFDAT,thisCdp,{})"
    return (retString.format(type,A[0].strip(),B))

#Function: Correct Operators
def corrOperators(code):
    #python operators (==,!=,>,<,>=,<=)
    #KRL operators (==, <>, >, <,>=, <=)
    code=code.replace("<>","!=")
    return code

#-------------------------------------------
# Variablen und Vereinbarungen
#-------------------------------------------

#Syntax DEF funcName
#Function: Define Python Function
parserLib["DEFDAT"]="ID_dataStart"
def ID_dataStart(code):
    retString="# " + code
    return (retString,0)

parserLib["ENDDAT"]="ID_dataEnd"
def ID_dataEnd(code):
    retString="# " + code
    return (retString,0)

#Syntax GLOBAL DEF funcName
#Function: Define Python Function
parserLib["GLOBAL DEF"]="ID_funcStartGlobal"
def ID_funcStartGlobal(code):
    retString="def " + code +":"
    return (retString,1)

#Syntax DEF funcName
#Function: Define Python Function
parserLib["DEF"]="ID_funcStart"
def ID_funcStart(code):
    retString="def " + code +":"
    return (retString,1)

#Syntax END
#Function: Decreace python indent
parserLib["END"]="ID_funcEnd"
def ID_funcEnd(code):
    return ("#EndFunction",-1)

#Syntax DECL {GLOBAL} Datentyp Name {= Wert} (S384)
#Function: define python variable
parserLib["DECL"]="ID_declaceVar"
def ID_declaceVar(code):
    
    #split Declaration and Value at "="
    A=code.split("=",1) #A0 Declaration, A1 Value
    #only use variable with value (aka A1 exists)
    if len(A)!=1:
        Dec=A[0].replace("GLOBAL","").strip()
        Dec=Dec.split()
        Val=A[1].strip("}{ ")
        Val=Val.replace("#","Hs")
        if Dec[0]=="E6POS" or Dec[0]=="FDAT" or Dec[0]=="PDAT":
            ValFormated="{"
            Val=Val.split(",")
            for Param in Val:
                temp=Param.strip()
                temp=Param.split(" ",1)
                temp2=temp[1].replace(".","").strip(" -")
                if temp2.isdigit():
                    ValFormated=ValFormated+"\""+temp[0].strip()+"\":" +temp[1].strip()+","
                else:
                    ValFormated=ValFormated+"\""+temp[0].strip()+"\":\"" +temp[1].strip()+"\","
            ValFormated=ValFormated[:-1]+"}"

            #Val=Val.replace(", ",",")
            #Val=Val.replace(" ,",",")
            #Val=Val.replace("{ ","{")
            #Val=Val.replace(" }","}")
            #Val=Val.replace("{","{\"")
            #Val=Val.replace(",",",\"")
            #Val=Val.replace(" ","\":")
            retString= Dec[1]+"="+ValFormated
        else:
            Val=Val.replace("\"","")
            Val=Val.replace("           "," ")
            retString= Dec[1]+"=\""+Val+"\""
    else:
        retString="#Var: "+ A
    return (retString,0)

# ENUM 4.2 "ENUM" 386 (NOT IMPLEMENTED)
# STRUC 4.3 "STRUC" 387 (NOT IMPLEMENTED)

#-------------------------------------------
# DAT FILE
#-------------------------------------------
#DEFDAT  UP_SERVICE
#DECL E6POS XService_VP={X -2045.92603,Y -6.20413113,Z 2966.34863,A 179.981964,B 0.0131895663,C -179.960907,S 18,T 10,E1 -3000.00098,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
#DECL FDAT FService_VP={TOOL_NO 15,BASE_NO 15,IPO_FRAME #BASE,POINT2[] " "}
#DECL PDAT PPService_VP={VEL 50.0000,ACC 100.000,APO_DIST 50.0000,APO_MODE #CDIS,GEAR_JERK 50.0000,EXAX_IGN 0}
#DECL E6POS XService_PP={X -1886.29614,Y -1152.45886,Z 936.626343,A 144.080551,B 0.890194833,C -89.3082886,S 18,T 34,E1 -3000.00098,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
#DECL FDAT FService_PP={TOOL_NO 15,BASE_NO 15,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}
#DECL PDAT PPService_PP={VEL 100.000,ACC 100.000,APO_DIST 0.0,GEAR_JERK 50.0000,EXAX_IGN 0}
#DECL E6POS XService_NP={X -2045.92603,Y -6.20413113,Z 2966.34863,A 179.981964,B 0.0131895663,C -179.960907,S 18,T 10,E1 -3000.00098,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
#DECL FDAT FService_NP={TOOL_NO 15,BASE_NO 15,IPO_FRAME #BASE,POINT2[] " "}
#DECL PDAT PPService_NP={VEL 50.0000,ACC 100.000,APO_DIST 50.0000,APO_MODE #CDIS,GEAR_JERK 50.0000,EXAX_IGN 0}
#DECL BASIS_SUGG_T LAST_BASIS={POINT1[] "P0                      ",POINT2[] "P0                      ",CP_PARAMS[] "CPDAT0                  ",PTP_PARAMS[] "PDAT0                   ",CONT[] "                        ",CP_VEL[] "2.0                     ",PTP_VEL[] "100                     ",SYNC_PARAMS[] "SYNCDAT                 ",SPL_NAME[] "S0                      ",A_PARAMS[] "ADAT0                   ",PC_SYNCSTRING[] "                        ",GL_SYNCSTRING[] "                        "}
#DECL MODULEPARAM_T LAST_TP_PARAMS={PARAMS[] "Kuka.PointName=Service_NP; Kuka.FrameData.base_no=15; Kuka.FrameData.tool_no=15; Kuka.FrameData.ipo_frame=#BASE; Kuka.FrameData.point2=; Kuka.isglobalpoint=False; Kuka.MoveDataPtpName=PService_NP; Kuka.MovementDataPdat.apo_mode=#CDIS; Kuka.MovementDataPdat.apo_dist=50; Kuka.MovementData.vel=50; Kuka.MovementData.acc=100; Kuka.MovementData.exax_ign=0; Kuka.VelocityPtp=50; Kuka.BlendingEnabled=True; Kuka.CurrentCDSetIndex=0    "}

#-------------------------------------------
# Bewegungsprogrammierung
#-------------------------------------------

parserLib["$BWDSTART"]="ID_setBwdstart"
#Syntax $BWDSTART=FALSE (currently unknown what this does)
#Function: ???
def ID_setBwdstart(code):
    A=getValueEquals(code)
    retString="#setBwdstart({})"
    return (retString.format(A),0)

parserLib["PDAT_ACT"]="ID_setPdatAct"
#Syntax PDAT_ACT=PPService_NP
#Function: Set PP value for movement
def ID_setPdatAct(code):
    A=getValueEquals(code)
    retString="thisPDAT={}"
    return (retString.format(A),0)

parserLib["FDAT_ACT"]="ID_setFdatAct"
#Syntax FDAT_ACT=FService_NP
#Function: Set F value for movement
def ID_setFdatAct(code):
    A=getValueEquals(code)
    retString="thisFDAT={}"
    return (retString.format(A),0)

parserLib["BAS"]="ID_setBase"
#Syntax BAS(#PTP_PARAMS, 50.0)
#Function: ???
def ID_setBase(code):
    A=code.split(",",1) #A0 = unknown, #A1=unknown
    retString="#setBase(\"{}\",{})"
    return (retString.format(A[0].strip(" ()"),A[1].strip(" ()")),0)

parserLib["SET_CD_PARAMS"]="ID_setCdParams"
def ID_setCdParams(code):
    A=code.strip(" ()")
    retString="thisCdp={}"
    return (retString.format(A),0)

parserLib["PTP"]="ID_movePtp"
#Syntax PTP XService_NP C_Dis (S 388)
#Function: call Move function with parameters
def ID_movePtp(code):
    A=getMoveParams("PTP",code)
    retString="KRL.move" + A
    return (retString,0)

parserLib["LIN"]="ID_moveLin"
#Syntax (S 390)
def ID_moveLin(code):
    retString="KRL.move" + getMoveParams("LIN",code)
    return (retString,0)

# PTP_REL 389 (NOT IMPLEMENTED)
# LIN_REL 391 (NOT IMPLEMENTED)
# SPLINE 396 (NOT IMPLEMENTED)
# PTP_SPLINE 397 (NOT IMPLEMENTED)
# SLIN 398 (NOT IMPLEMENTED)
# SLIN_REL 399 (NOT IMPLEMENTED)
# SPTP 401 (NOT IMPLEMENTED)
# SPTP_REL 402 (NOT IMPLEMENTED)
# TIME_BLOCK 404 (NOT IMPLEMENTED)
# CONST_VEL 406 (NOT IMPLEMENTED)
# STOP WHEN PATH 409 (NOT IMPLEMENTED)

#-------------------------------------------
# Programmablaufkontrolle
#-------------------------------------------

# CONTINUE 411
# --> IMPLEMENT IN FUTURE?

# EXIT 7.2 "EXIT" 412 (NOT IMPLEMENTED)

parserLib["FOR"]="ID_forOp"
#Syntax FOR … TO … ENDFOR 7.3 "FOR ... TO ... ENDFOR" 412
def ID_forOp(code):
    A=code.split("TO")
    retString="for {} in range({}):"
    return (retString.format(A[0].strip(),A[1].strip()),1)

parserLib["ENDFOR"]="ID_endForOp"
def ID_endForOp(code):
    return ("#EndFor",-1)

# GOTO 7.4 "GOTO" 413 (NOT IMPLEMENTED)

parserLib["HALT"]="ID_haltOp"
#Syntax S414
#Function pause program untill user says continue
def ID_haltOp(code):
    retString="msg=input(\"Press Enter to contine\")"
    return (retString,0)

parserLib["IF"]="ID_ifOp"
#Syntax IF … THEN … ENDIF 7.6 "IF ... THEN ... ENDIF" 414
def ID_ifOp(code):
    code=corrOperators(code)
    A=code.replace("THEN","").strip()
    retString="if {}:"
    return (retString.format(A),1)

parserLib["ELSE"]="ID_elseOp"
def ID_elseOp(code):
    return ("else:",10)

parserLib["ENDIF"]="ID_endifOp"
def ID_endifOp(code):
    return ("#EndIf:",-1)

parserLib["LOOP"]="ID_loopOp"
#Syntax LOOP … ENDLOOP 415
def ID_loopOp(code):
    return ("while True:",1)

parserLib["ENDLOOP"]="ID_endloopOp"
def ID_endloopOp(code):
    return ("#Endwhile",-1)

# ON_ERROR_PROCEED 415 (NOT IMPLEMENTED)
# REPEAT … UNTIL 421 (NOT IMPLEMENTED)

parserLib["SWITCH"]="ID_switchOp"
#Syntax SWITCH … CASE … ENDSWITCH 421
def ID_switchOp(code):
    global firstInSwitch
    firstInSwitch=True
    retString="switch="+code
    return (retString,0)

parserLib["CASE"]="ID_caseOp"
def ID_caseOp(code):
    global firstInSwitch
    if firstInSwitch: indent=1 #just entered switch
    else: indent=10
    firstInSwitch=False
    retString="if switch=="+code+":"
    return (retString,indent)

parserLib["DEFAULT"]="ID_defaultOp"
def ID_defaultOp(code):
    global firstInSwitch
    if firstInSwitch: indent=1 #just entered switch
    else: indent=10
    firstInSwitch=False
    return ("else:",10)

parserLib["ENDSWITCH"]="ID_endswitchOp"
def ID_endswitchOp(code):
    return ("#EndSwitch",-1)

# WAIT FOR … 423 (NOT IMPLEMENTED)

parserLib["WAIT SEC"]="ID_waitSec"
#Syntax WAIT SEC … 423
def ID_waitSec(code):
    return "time.sleep(float(code))"

parserLib["WHILE"]="ID_whileOp"
#Syntax WHILE … ENDWHILE 424
def ID_whileOp(code):
    code=corrOperators(code)
    retString="while "+code+":"
    return (retString,1)

parserLib["ENDWHILE"]="ID_endwhileOp"
def ID_endwhileOp(code):
    return ("#EndWhile",-1)

# Ein-/Ausgänge
# ANIN 8.1 "ANIN" 424 (NOT IMPLEMENTED)
# ANOUT 8.2 "ANOUT" 425 (NOT IMPLEMENTED)
# PULSE 8.3 "PULSE" 426 (NOT IMPLEMENTED)
# SIGNAL 8.4 "SIGNAL" 430 (NOT IMPLEMENTED)

#-------------------------------------------
# Unterprogramme und Funktionen
#-------------------------------------------
# DEFFCT … ENDFCT  432
# --> IMPLEMENT IN FUTURE?

# return 432
# --> IMPLEMENT IN FUTURE?

#-------------------------------------------
# Interrupt-Programmierung
#-------------------------------------------
# BRAKE 10.1 "BRAKE" 438 (NOT IMPLEMENTED)
# INTERRUPT 10.3 "INTERRUPT" 441 (NOT IMPLEMENTED)
# INTERRUPT … DECL … WHEN … DO 439 (NOT IMPLEMENTED)
# RESUME 10.4 (NOT IMPLEMENTED)

#-------------------------------------------
# Bahnbezogene Schaltaktionen (=Trigger
#-------------------------------------------
parserLib["TRIGGER WHEN"]="ID_triggerWhen"
#Syntax TRIGGER WHEN DISTANCE/PATH 444/447
def ID_triggerWhen(code):
    #TRIGGER WHEN DISTANCE=Position DELAY=Zeit DO Anweisung <PRIO=Prioritaet>
    A=code.split("=",1) #A0 = Mode
    A[0]=A[0].lower()
    B=A[1].split("DELAY=",1) #B0 = Value
    C=B[1].split("DO") #C0 = Delay
    D=C[1].split("PRIO=") #D0 = Function, D1 (if exists) = Priority
    #p={"M":A[0].strip(), "Val":B[0].strip() , "Del":C[0].strip() , "Func":D[0].strip()}
    retString="KRL.triggerWhen(\"{}\",{},{},\"{}\")"
    return (retString.format(A[0].strip(),B[0].strip(),C[0].strip(),D[0].strip()),0)
    
#-------------------------------------------
# Kommunikation "Kommunikation" 455
#-------------------------------------------

#-------------------------------------------
# Operatoren
#-------------------------------------------
# Arithmetische Operatoren 13.1 "Arithmetische Operatoren" 456
# Geometrischer Operator 13.2 "Geometrischer Operator" 457
# Vergleichsoperatoren 13.3 "Vergleichsoperatoren" 460
# Logische Operatoren 13.4 "Logische Operatoren" 461
# Bit-Operatoren 13.5 "Bit-Operatoren" 461
# Prioritäten der Operatoren 13.6 "Priorität der Operatoren" 464

#-------------------------------------------
# Systemfunktionen
#-------------------------------------------
# DELETE_BACKWARD_BUFFER( 14.1 "DELETE_BACKWARD_BUFFER(" 464
# ROB_STOP( 14.2 "ROB_STOP( und
# ROB_STOP_RELEASE(" 465
# SET_BRAKE_DELAY( 14.3 "SET_BRAKE_DELAY(" 466
# VARSTATE( 14.4 "VARSTATE(" 469
# Stringvariablen manipulieren "Stringvariablen bearbeiten" 471

print(parserLib)