"""
Variablen und Vereinbarungen
DECL 4.1 "DECL" 384
ENUM 4.2 "ENUM" 386
STRUC 4.3 "STRUC" 387

Bewegungsprogrammierung
PTP 5.1 "PTP" 388
LIN, CIRC 5.3 "LIN, CIRC" 390
PTP_REL 5.2 "PTP_REL" 389
LIN_REL, CIRC_REL 5.4 "LIN_REL, CIRC_REL" 391
SPLINE … ENDSPLINE 6.1 "SPLINE ... ENDSPLINE" 396
PTP_SPLINE … ENDSPLINE 6.2 "PTP_SPLINE ... ENDSPLINE" 397
SLIN, SCIRC, SPL 6.3 "SLIN, SCIRC, SPL" 398
SLIN_REL, SCIRC_REL, SPL_REL 6.4 "SLIN_REL, SCIRC_REL, SPL_REL" 399
SPTP 6.5 "SPTP" 401
SPTP_REL 6.6 "SPTP_REL" 402
TIME_BLOCK 6.8 "TIME_BLOCK" 404
CONST_VEL 6.9 "CONST_VEL" 406
STOP WHEN PATH 6.10 "STOP WHEN PATH" 409

Programmablaufkontrolle
CONTINUE 7.1 "CONTINUE" 411
EXIT 7.2 "EXIT" 412
FOR … TO … ENDFOR 7.3 "FOR ... TO ... ENDFOR" 412
GOTO 7.4 "GOTO" 413
HALT 7.5 "HALT" 414
IF … THEN … ENDIF 7.6 "IF ... THEN ... ENDIF" 414
LOOP … ENDLOOP 7.7 "LOOP ... ENDLOOP" 415
ON_ERROR_PROCEED 7.8 "ON_ERROR_PROCEED" 415
REPEAT … UNTIL 7.9 "REPEAT ... UNTIL" 421
SWITCH … CASE … ENDSWITCH 7.10 "SWITCH ... CASE ... ENDSWITCH" 421
WAIT FOR … 7.11 "WAIT FOR …" 423
WAIT SEC … 7.12 "WAIT SEC …" 423
WHILE … ENDWHILE 7.13 "WHILE ... ENDWHILE" 424

Ein-/Ausgänge
ANIN 8.1 "ANIN" 424
ANOUT 8.2 "ANOUT" 425
PULSE 8.3 "PULSE" 426
SIGNAL 8.4 "SIGNAL" 430

Unterprogramme und Funktionen
DEFFCT … ENDFCT 9.3 "DEFFCT ... ENDFCT" 432
RETURN 9.4 "RETURN" 432

Interrupt-Programmierung
BRAKE 10.1 "BRAKE" 438
INTERRUPT 10.3 "INTERRUPT" 441
INTERRUPT … DECL … WHEN … DO 10.2 "INTERRUPT ... DECL ... WHEN ... DO" 439
RESUME 10.4 "RESUME" 443

Bahnbezogene Schaltaktionen (=Trigger
TRIGGER WHEN DISTANCE 11.1 "TRIGGER WHEN DISTANCE" 444
TRIGGER WHEN PATH 11.2 "TRIGGER WHEN PATH" 447

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


#"TRIGGER WHEN":"triggerWhen" #444
def triggerWhen(code):
    #TRIGGER WHEN DISTANCE=Position DELAY=Zeit DO Anweisung <PRIO=Prioritaet>
    A=code.split("=",1)
    B=A[1].split("DELAY=",1)
    C=B[1].split("DO")
    D=C[1].split("PRIO=")
    params={"Mode":A[0].strip(), "Value":B[0].strip() , "Delay":C[0].strip() , "Func":D[0].strip()}
    print(params)

#"$BWDSTART"
def setBwdstart(code):
    pass

#"PDAT_ACT"
def setPdatAct(code):
    pass

#"FDAT_ACT"
def setFdatAct(code):
    pass

#"BAS"
def setBase(code):
    pass

#"SET_CD_PARAMS"
def setCdParams(code):
    pass

#"PTP"
def movePtp(code):
    pass

#"CONTINUE"
def continueOp(code):
    pass

#"WHILE"
def whileOp(code):
    pass
#"SWITCH"
def switchOp(code):
    pass

#"CASE"
def caseOp(code):
    pass

#"DEFAULT"
def defaultOp(code):
    pass

#"LOOP"
def loopOp(code):
    pass

#"HALT"
def halt(code):
    pass

#"ENDLOOP"
def endloopOp(code):
    pass

#"ENDSWITCH"
def endswitchOp(code):
    pass

#"ENDWHILE"
def endwhileOp(code):
    pass

#"END"
def funcEnd(code):
    pass

#"GLOBAL DEF"
def funcStartGlobal(code):
    pass

#"DEF"
def funcStart(code):
    pass