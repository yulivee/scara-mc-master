#-------------------------------------------
# Function and Variables Definitions
#-------------------------------------------

def getSingleValue(code):
    A=code.split("=",1) #A1 = Value
    return A[1].strip()

#non zero if programm is in a switch case
firstInSwitch=True

#-------------------------------------------
# Variablen und Vereinbarungen
#-------------------------------------------

#"GLOBAL DEF"
def ID_funcStartGlobal(code):
    retString="def " + code +":"
    return (retString,1)

#"DEF"
def ID_funcStart(code):
    retString="def " + code +":"
    return (retString,1)

#"END"
def ID_funcEnd(code):
    return ("#EndFunction",-1)

# DECL 4.1 "DECL" 384
# ENUM 4.2 "ENUM" 386
# STRUC 4.3 "STRUC" 387

#-------------------------------------------
# Bewegungsprogrammierung
#-------------------------------------------

#"$BWDSTART"
def ID_setBwdstart(code):
    #$BWDSTART=FALSE (currently unknown what this does)
    retString="KRL.setBwdstart({})"
    return (retString.format(getSingleValue(code)),0)

def setBwdstart(code):
    print("setBwdstart")

#"PDAT_ACT"
def ID_setPdatAct(code):
    #PDAT_ACT=PPService_NP
    retString="KRL.setPdatAct({})"
    return (retString.format(getSingleValue(code)),0)

#"FDAT_ACT"
def ID_setFdatAct(code):
    #FDAT_ACT=FService_NP
    retString="KRL.setFdatAct({})"
    return (retString.format(getSingleValue(code)),0)

#"BAS"
def ID_setBase(code):
    #BAS(#PTP_PARAMS, 50.0)
    A=code.split(",",1) #A0 = unknown, #A1=unknown
    retString="KRL.setBase(\"{}\",{})"
    return (retString.format(A[0].strip(" ()"),A[1].strip(" ()")),0)

#"SET_CD_PARAMS"
def ID_setCdParams(code):
    code
    retString="KRL.setCdParams({})"
    return (retString.format(code.strip(" ()")),0)

#"PTP"
def ID_movePtp(code):
    #PTP XService_NP C_Dis
    A=code.split(" ",1) #A0 = Value, #A1=flyby (optional)
    if len(A)==1:
        B=True
    else:
        B=False
    retString="KRL.movePtp({},{})"
    return (retString.format(A[0].strip(),B),0)

# PTP 5.1 "PTP" 388
# LIN, CIRC 5.3 "LIN, CIRC" 390
# PTP_REL 5.2 "PTP_REL" 389
# LIN_REL, CIRC_REL 5.4 "LIN_REL, CIRC_REL" 391
# SPLINE … ENDSPLINE 6.1 "SPLINE ... ENDSPLINE" 396
# PTP_SPLINE … ENDSPLINE 6.2 "PTP_SPLINE ... ENDSPLINE" 397
# SLIN, SCIRC, SPL 6.3 "SLIN, SCIRC, SPL" 398
# SLIN_REL, SCIRC_REL, SPL_REL 6.4 "SLIN_REL, SCIRC_REL, SPL_REL" 399
# SPTP 6.5 "SPTP" 401
# SPTP_REL 6.6 "SPTP_REL" 402
# TIME_BLOCK 6.8 "TIME_BLOCK" 404
# CONST_VEL 6.9 "CONST_VEL" 406
# STOP WHEN PATH 6.10 "STOP WHEN PATH" 409
#
# Programmablaufkontrolle
# CONTINUE 7.1 "CONTINUE" 411
# EXIT 7.2 "EXIT" 412
# FOR … TO … ENDFOR 7.3 "FOR ... TO ... ENDFOR" 412
# GOTO 7.4 "GOTO" 413
# HALT 7.5 "HALT" 414
#"HALT"
def ID_haltOp(code):
    retString="msg=input(\"Press Enter to contine\")"
    return (retString,0)

# IF … THEN … ENDIF 7.6 "IF ... THEN ... ENDIF" 414

# LOOP … ENDLOOP 7.7 "LOOP ... ENDLOOP" 415
#"LOOP"
def ID_loopOp(code):
    return ("while True:",1)

#"ENDLOOP"
def ID_endloopOp(code):
    return ("#Endwhile",-1)

# ON_ERROR_PROCEED 7.8 "ON_ERROR_PROCEED" 415
# REPEAT … UNTIL 7.9 "REPEAT ... UNTIL" 421

# SWITCH … CASE … ENDSWITCH 7.10 "SWITCH ... CASE ... ENDSWITCH" 421
#"SWITCH"
def ID_switchOp(code):
    global firstInSwitch
    firstInSwitch=True
    retString="switch="+code
    return (retString,0)

#"CASE"
def ID_caseOp(code):
    global firstInSwitch
    if firstInSwitch: indent=1 #just entered switch
    else: indent=10
    firstInSwitch=False
    retString="if switch=="+code+":"
    return (retString,indent)

#"DEFAULT"
def ID_defaultOp(code):
    global firstInSwitch
    if firstInSwitch: indent=1 #just entered switch
    else: indent=10
    firstInSwitch=False
    return ("else:",10)

#"ENDSWITCH"
def ID_endswitchOp(code):
    return ("#EndSwitch",-1)

# WAIT FOR … 7.11 "WAIT FOR …" 423
# WAIT SEC … 7.12 "WAIT SEC …" 423

# WHILE … ENDWHILE 7.13 "WHILE ... ENDWHILE" 424
#"WHILE"
def ID_whileOp(code):
    retString="while "+code+":"
    return (retString,1)

#"ENDWHILE"
def ID_endwhileOp(code):
    return ("#EndWhile",-1)

#
# Ein-/Ausgänge
# ANIN 8.1 "ANIN" 424
# ANOUT 8.2 "ANOUT" 425
# PULSE 8.3 "PULSE" 426
# SIGNAL 8.4 "SIGNAL" 430
#
# Unterprogramme und Funktionen
# DEFFCT … ENDFCT 9.3 "DEFFCT ... ENDFCT" 432
# return (9.4 "RETURN" 432
#
# Interrupt-Programmierung
# BRAKE 10.1 "BRAKE" 438
# INTERRUPT 10.3 "INTERRUPT" 441
# INTERRUPT … DECL … WHEN … DO 10.2 "INTERRUPT ... DECL ... WHEN ... DO" 439
# RESUME 10.4 "RESUME" 443
#
# Bahnbezogene Schaltaktionen (=Trigger
# TRIGGER WHEN DISTANCE 11.1 "TRIGGER WHEN DISTANCE" 444
# TRIGGER WHEN PATH 11.2 "TRIGGER WHEN PATH" 447
#
#"TRIGGER WHEN":"triggerWhen" #444
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
    
"""
Kommunikation "Kommunikation" 455

Operatoren
Arithmetische Operatoren 13.1 "Arithmetische Operatoren" 456
Geometrischer Operator 13.2 "Geometrischer Operator" 457
Vergleichsoperatoren 13.3 "Vergleichsoperatoren" 460
Logische Operatoren 13.4 "Logische Operatoren" 461
Bit-Operatoren 13.5 "Bit-Operatoren" 461
Prioritäten der Operatoren 13.6 "Priorität der Operatoren" 464

Systemfunktionen
DELETE_BACKWARD_BUFFER( 14.1 "DELETE_BACKWARD_BUFFER(" 464
ROB_STOP( 14.2 "ROB_STOP( und
ROB_STOP_RELEASE(" 465
SET_BRAKE_DELAY( 14.3 "SET_BRAKE_DELAY(" 466
VARSTATE( 14.4 "VARSTATE(" 469

Stringvariablen manipulieren "Stringvariablen bearbeiten" 471
"""