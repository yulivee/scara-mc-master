#Temp PY Robot Programm
import KRL
import time
#&ACCESS RVO
#&REL 7
#&PARAM DISKPATH = KRC:\R1\Program\Unterprogramme
# UP_SERVICE
#INT job
XService_VP={"X":-2045.92603,"Y":-6.20413113,"Z":2966.34863,"A":179.981964,"B":0.0131895663,"C":-179.960907,"S":18,"T":10,"E1":-3000.00098,"E2":0.0,"E3":0.0,"E4":0.0,"E5":0.0,"E6":0.0}
FService_VP={"TOOL_NO":15,"BASE_NO":15,"IPO_FRAME":"HsBASE","POINT2[]":"" ""}
PPService_VP={"VEL":50.0000,"ACC":100.000,"APO_DIST":50.0000,"APO_MODE":"HsCDIS","GEAR_JERK":50.0000,"EXAX_IGN":0}
XService_PP={"X":-1886.29614,"Y":-1152.45886,"Z":936.626343,"A":144.080551,"B":0.890194833,"C":-89.3082886,"S":18,"T":34,"E1":-3000.00098,"E2":0.0,"E3":0.0,"E4":0.0,"E5":0.0,"E6":0.0}
FService_PP={"TOOL_NO":15,"BASE_NO":15,"IPO_FRAME":"HsBASE","POINT2[]":"" "","TQ_STATE":"FALSE"}
PPService_PP={"VEL":100.000,"ACC":100.000,"APO_DIST":0.0,"GEAR_JERK":50.0000,"EXAX_IGN":0}
XService_NP={"X":-2045.92603,"Y":-6.20413113,"Z":2966.34863,"A":179.981964,"B":0.0131895663,"C":-179.960907,"S":18,"T":10,"E1":-3000.00098,"E2":0.0,"E3":0.0,"E4":0.0,"E5":0.0,"E6":0.0}
FService_NP={"TOOL_NO":15,"BASE_NO":15,"IPO_FRAME":"HsBASE","POINT2[]":"" ""}
PPService_NP={"VEL":50.0000,"ACC":100.000,"APO_DIST":50.0000,"APO_MODE":"HsCDIS","GEAR_JERK":50.0000,"EXAX_IGN":0}
LAST_BASIS="POINT1[] P0  ,POINT2[] P0  ,CP_PARAMS[] CPDAT0        ,PTP_PARAMS[] PDAT0         ,CONT[]     ,CP_VEL[] 2.0           ,PTP_VEL[] 100           ,SYNC_PARAMS[] SYNCDAT       ,SPL_NAME[] S0  ,A_PARAMS[] ADAT0         ,PC_SYNCSTRING[]     ,GL_SYNCSTRING[]     "
LAST_TP_PARAMS="PARAMS[] Kuka.PointName=Service_NP"
# 
#&ACCESS RVO
#&REL 7
def UP_Service():
 #job = 6
 #VZ [Job] = {b FALSE, IDX 0}
 #SRC_INIT()
 #AuftragStart()
 KRL.triggerWhen("distance",1,0,"JobNr= Job")
 KRL.triggerWhen("distance",1,0,"PosNr= 41")
 #setBwdstart(FALSE)
 thisPDAT=PPService_VP
 thisFDAT=FService_VP
 #setBase("#PTP_PARAMS",50.0)
 thisCdp=0
 KRL.move("PTP",XService_VP,thisPDAT,thisFDAT,thisCdp,False)
 #Continue
 #WAIT_FOR_INPUT_M(eFrg[job].Ef[], TRUE)
 #Continue
 #Set_CYCFLAG(eFrg[job].Ef[],M_Inbetriebnahme)
 KRL.triggerWhen("distance",0,0,"Reset_(aFrg[job].Ab[])")
 KRL.triggerWhen("distance",1,0,"PosNr= 45")
 #setBwdstart(FALSE)
 thisPDAT=PPService_PP
 thisFDAT=FService_PP
 #setBase("#PTP_PARAMS",50)
 thisCdp=0
 KRL.move("PTP",XService_PP,thisPDAT,thisFDAT,thisCdp,True)
 #Handshake(aFrg[job].qEf[],eFrg[job].Ef[],eFrg[job].Pr[])
 #Handshake(aFrg[job].qPr[],eFrg[job].Pr[],eFrg[job].Rf[],OhneCyc,JobEnde)
 KRL.triggerWhen("distance",1,0,"SetAB(aFrg[job].Ab[])")
 KRL.triggerWhen("distance",0,0,"PosNr= 49")
 #setBwdstart(FALSE)
 thisPDAT=PPService_NP
 thisFDAT=FService_NP
 #setBase("#PTP_PARAMS",50.0)
 thisCdp=0
 KRL.move("PTP",XService_NP,thisPDAT,thisFDAT,thisCdp,False)
 #CONTINUE
#EndFunction
def Rueckzug_Service():
 while PosNr!= 10:
  switch=PosNr
  if switch==41:
   KRL.triggerWhen("distance",1,0,"PosNr= 10")
   #MoveParamSet(FService_VP,RueckZugAcc,RueckZugVel)
   KRL.move("PTP",XService_VP,thisPDAT,thisFDAT,thisCdp,True)
  if switch==45:
   #PosNr= 41
  if switch==49:
   KRL.triggerWhen("distance",1,0,"PosNr= 10")
   #MoveParamSet(FService_NP,RueckZugAcc,RueckZugVel)
   KRL.move("PTP",XService_NP,thisPDAT,thisFDAT,thisCdp,True)
  else:
   while True:
    msg=input("Press Enter to contine")
   #Endwhile
  #EndSwitch
 #EndWhile
 #CONTINUE
#EndFunction
