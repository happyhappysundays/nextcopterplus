/*
 GUI understands following command strings:
 M  Multiwii @ arduino send all data to GUI (former A)
 W  write to Eeprom @ arduino (former C)
 S  acc Sensor calibration request(former D)
 E  mag Sensor calibration request
 C  Stick centering request
 D  Load default values request
*/

import processing.serial.*; // serial library
import controlP5.*; // controlP5 library
import processing.opengl.*;

Serial g_serial;
ControlP5 controlP5;
Textlabel txtlblWhichcom,version; 
ListBox commListbox;

//int frame_size = 115; // Number of serial data bytes
//int frame_size = 65;
int frame_size = 69; // Beta 3 frame size

cGraph g_graph;
int windowsX    = 780; int windowsY    = 470;
int xGraph      = 10;  int yGraph      = 255;
//int xObj        = 670; int yObj        = 410;
int xObj        = 720; int yObj        = 500;
int xParam      = 120; int yParam      = 10;
int xRC         = 630; int yRC         = 15;
int xMot        = 480; int yMot        = 30;

int xButton   = 650;         int yButton    = 150;
int xBox      = xParam+190;  int yBox      = yParam+120;


boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,baroGraph=true,magGraph=true;
boolean magxGraph =true,magyGraph=true,magzGraph=true;
boolean debug1Graph = false,debug2Graph = false,debug3Graph = false,debug4Graph = false;

int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX, ...
int present=0,mode=0;

cDataArray accPITCH   = new cDataArray(100), accROLL    = new cDataArray(100), accYAW     = new cDataArray(100);
cDataArray gyroPITCH  = new cDataArray(100), gyroROLL   = new cDataArray(100), gyroYAW    = new cDataArray(100);
cDataArray magxData   = new cDataArray(100), magyData   = new cDataArray(100), magzData   = new cDataArray(100);
cDataArray baroData   = new cDataArray(100);
cDataArray magData    = new cDataArray(100);
cDataArray debug1Data   = new cDataArray(100),debug2Data   = new cDataArray(100),debug3Data   = new cDataArray(100),debug4Data   = new cDataArray(100);

private static final int ROLL = 0, PITCH = 1, YAW = 2, ALT = 3, VEL = 4, LEVEL = 5, MAG = 6;

Numberbox confP[] = new Numberbox[7], confI[] = new Numberbox[7], confD[] = new Numberbox[7];
Numberbox confRC_RATE, confRC_EXPO;
Numberbox rollPitchRate,yawRate,AccRollTrim,AccPitchTrim;

int byteP[] = new int[7], byteI[] = new int[7],byteD[] = new int[7];

int  byteRC_RATE,byteRC_EXPO,
     byteRollPitchRate,byteYawRate,
     byteDynThrPID;
int  byteAccRollTrim=0, byteAccPitchTrim=0;
//float  byteAccRollTrim, byteAccPitchTrim;

Slider rcStickThrottleSlider,rcStickRollSlider,rcStickPitchSlider,rcStickYawSlider;
Slider rcStickAUX1Slider,rcStickAUX2Slider,rcStickAUX3Slider,rcStickAUX4Slider;

Slider motSliderV0,motSliderV1,motSliderV2,motSliderV3,motSliderV4,motSliderV5;
Slider servoSliderH1,servoSliderH2,servoSliderH3,servoSliderH4;
Slider servoSliderV0,servoSliderV1,servoSliderV2;

Slider axSlider,aySlider,azSlider,gxSlider,gySlider,gzSlider;
Slider magxSlider,magySlider,magzSlider;
Slider baroSlider,magSlider;
Slider debug1Slider,debug2Slider,debug3Slider,debug4Slider;

Slider scaleSlider, batSlider;

Button buttonREAD,buttonWRITE,buttonCALIBRATE_ACC,buttonCALIBRATE_MAG,buttonSTART,buttonSTOP;
Button buttonDEFAULTS,buttonSTICKS,buttonLVAMODE,buttonKK;

Button buttonNunchuk,buttonI2cAcc,buttonI2cBaro,buttonI2cMagneto;
Button buttonI2cAccActive,buttonI2cBaroActive,buttonI2cMagnetoActive,buttonAutolevelActive;

color yellow_ = color(200, 200, 20), green_ = color(30, 120, 30), red_ = color(120, 30, 30), grey_ = color(60, 60, 60), orange_ = color(255, 150, 0);
boolean graphEnable = false;boolean readEnable = false;boolean writeEnable = false;boolean calibrateEnable = false;boolean KKEnable = false;

float gx,gy,gz,ax,ay,az,magx,magy,magz,baro,mag,angx,angy,debug1,debug2,debug3,debug4;
int init_com,graph_on,intPowerTrigger,bytevbat;
float pMeterSum; //debug

Numberbox confPowerTrigger;

float mot[] = new float[8];

float servo0=1500,servo1=1500,servo2=1500,servo3=1500;
float rcThrottle = 1500,rcRoll = 1500,rcPitch = 1500,rcYaw =1500,
      rcAUX1=1500, rcAUX2=1500, rcAUX3=1500, rcAUX4=1500;
int AutoLevelON,NormalMode,AcroMode,StabilityMode,levelMode,LVAMode;
boolean LVAButtonMode = false;

float time1,time2,time3;
int cycleTime;


private static final int BOXACC = 0,BOXBARO = 1,BOXMAG = 2,BOXCAMSTAB = 3,BOXCAMTRIG = 4,BOXARM = 5;

CheckBox checkbox[] = new CheckBox[6];
int activation[] = new int[6];

PFont font8,font12,font15;

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
String shortifyPortName(String portName, int maxlen)  {
  String shortName = portName;
  if(shortName.startsWith("/dev/"))
    shortName = shortName.substring(5);  

  if(shortName.startsWith("tty.")) // get rid off leading tty. part of device name
    shortName = shortName.substring(4); 

  if(portName.length()>maxlen) {
    shortName = shortName.substring(0,(maxlen-1)/2) + "~" +shortName.substring(shortName.length()-(maxlen-(maxlen-1)/2));
  }
  if(shortName.startsWith("cu.")) // only collect the corresponding tty. devices
    shortName = "";
  return shortName;
}

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}

void setup() {
  size(windowsX,windowsY,OPENGL);
  frameRate(20); 

  font8 = createFont("Arial bold",8,false);font12 = createFont("Arial bold",12,false);font15 = createFont("Arial bold",15,false);
  //font8 = createFont("Tahoma",8,false);font12 = createFont("Tahoma",10,false);font15 = createFont("Tahoma",15,false);
  //font8 = loadFont("Tahoma-8.vlw");font12 = loadFont("Tahoma-10.vlw");font15 = loadFont("Tahoma-15.vlw");
  
  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(font12);

  g_graph  = new cGraph(xGraph+110,yGraph, 480, 200);
  // make a listbox and populate it with the available comm ports
  commListbox = controlP5.addListBox("portComList",5,80,110,240); //addListBox(name,x,y,width,height)

  commListbox.captionLabel().set("PORT COM");
  commListbox.setColorBackground(red_);
  for(int i=0;i<Serial.list().length;i++) {
    String pn = shortifyPortName(Serial.list()[i], 13);
    if (pn.length() >0 ) commListbox.addItem(pn,i); // addItem(name,value)
  }
  // text label for which comm port selected
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",8,55); // textlabel(name,text,x,y)
    
  buttonSTART = controlP5.addButton("bSTART",1,xGraph+110,yGraph-25,45,19); buttonSTART.setLabel("START"); buttonSTART.setColorBackground(red_);
  buttonSTOP = controlP5.addButton("bSTOP",1,xGraph+165,yGraph-25,40,19); buttonSTOP.setLabel("STOP"); buttonSTOP.setColorBackground(red_);

  buttonNunchuk = controlP5.addButton("bNUNCHUK",1,xParam+220,yParam+20,120,30);buttonNunchuk.setColorBackground(red_);buttonNunchuk.setLabel("       AUTOLEVEL");
  buttonI2cMagneto = controlP5.addButton("bMAG",1,xParam+220,yParam+55,120,30); buttonI2cMagneto.setColorBackground(red_);buttonI2cMagneto.setLabel("       STABILISED");  
  buttonI2cAcc = controlP5.addButton("bACC",1,xParam+220,yParam+90,120,30); buttonI2cAcc.setColorBackground(grey_);buttonI2cAcc.setLabel("      AUX mode 1");
  buttonI2cBaro = controlP5.addButton("bBARO",1,xParam+220,yParam+125,120,30); buttonI2cBaro.setColorBackground(grey_);buttonI2cBaro.setLabel("    Gain settings");

  
  color c,black;
  black = color(0,0,0);
  int xo = xGraph-7;
  int x = xGraph+40;
  int y1= yGraph+10;
  int y2= yGraph+100;
//  int y2= yGraph+60;
  int y5= yGraph+110;
  int y3= yGraph+160;
  
  int y4= yGraph+180;
  int y6= yGraph+205;

  
  Toggle tACC_ROLL =     controlP5.addToggle("ACC_ROLL",true,x,y1+10,20,20);tACC_ROLL.setColorActive(color(255, 0, 0));tACC_ROLL.setColorBackground(black);tACC_ROLL.setLabel(""); 
  Toggle tACC_PITCH =   controlP5.addToggle("ACC_PITCH",true,x,y1+30,20,20);tACC_PITCH.setColorActive(color(0, 255, 0));tACC_PITCH.setColorBackground(black);tACC_PITCH.setLabel(""); 
  Toggle tACC_Z =           controlP5.addToggle("ACC_Z",true,x,y1+50,20,20);tACC_Z.setColorActive(color(0, 0, 255));tACC_Z.setColorBackground(black);tACC_Z.setLabel(""); 
  Toggle tGYRO_ROLL =   controlP5.addToggle("GYRO_ROLL",true,x,y2+10,20,20);tGYRO_ROLL.setColorActive(color(200, 200, 0));tGYRO_ROLL.setColorBackground(black);tGYRO_ROLL.setLabel(""); 
  Toggle tGYRO_PITCH = controlP5.addToggle("GYRO_PITCH",true,x,y2+30,20,20);tGYRO_PITCH.setColorActive(color(0, 255, 255));tGYRO_PITCH.setColorBackground(black);tGYRO_PITCH.setLabel(""); 
  Toggle tGYRO_YAW =     controlP5.addToggle("GYRO_YAW",true,x,y2+50,20,20);tGYRO_YAW.setColorActive(color(255, 0, 255));tGYRO_YAW.setColorBackground(black);tGYRO_YAW.setLabel(""); 

  
  controlP5.addTextlabel("acclabel","MISC",xo,y1-80);
  controlP5.addTextlabel("acclabel","   VBAT",xo,y1-65);
  controlP5.addTextlabel("acclabel","   LVA",xo,y1-45);
  controlP5.addTextlabel("acclabel","   MODE",xo,y1-25);
  
  controlP5.addTextlabel("acclabel","ACC",xo,y1);
  controlP5.addTextlabel("accrolllabel","   ROLL",xo,y1+18);
  controlP5.addTextlabel("accpitchlabel","   PITCH",xo,y1+38);
  controlP5.addTextlabel("acczlabel","   Z",xo,y1+58);
  controlP5.addTextlabel("gyrolabel","GYRO",xo,y2);
  controlP5.addTextlabel("gyrorolllabel","   ROLL",xo,y2+17);
  controlP5.addTextlabel("gyropitchlabel","   PITCH",xo,y2+37);
  controlP5.addTextlabel("gyroyawlabel","   YAW",xo,y2+57);

  axSlider   =         controlP5.addSlider("axSlider",-1000,+1000,0,x+20,y1+10,50,20);axSlider.setDecimalPrecision(0);axSlider.setLabel("");
  aySlider   =         controlP5.addSlider("aySlider",-1000,+1000,0,x+20,y1+30,50,20);aySlider.setDecimalPrecision(0);aySlider.setLabel("");
  azSlider   =         controlP5.addSlider("azSlider",-1000,+1000,0,x+20,y1+50,50,20);azSlider.setDecimalPrecision(0);azSlider.setLabel("");
  gxSlider   =         controlP5.addSlider("gxSlider",-500,+500,0,x+20,y2+10,50,20);gxSlider.setDecimalPrecision(0);gxSlider.setLabel("");
  gySlider   =         controlP5.addSlider("gySlider",-500,+500,0,x+20,y2+30,50,20);gySlider.setDecimalPrecision(0);gySlider.setLabel("");
  gzSlider   =         controlP5.addSlider("gzSlider",-500,+500,0,x+20,y2+50,50,20);gzSlider.setDecimalPrecision(0);gzSlider.setLabel("");

  confP[ROLL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_ROLL",0,xParam+40,yParam+20,30,14));confP[ROLL].setDecimalPrecision(0);confP[ROLL].setMultiplier(0.1);confP[ROLL].setMax(255);
  confI[ROLL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_ROLL",0,xParam+75,yParam+20,45,14));confI[ROLL].setDecimalPrecision(3);confI[ROLL].setMultiplier(0);confI[ROLL].setMax(0.255);
  confD[ROLL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_ROLL",0,xParam+125,yParam+20,30,14));confD[ROLL].setDecimalPrecision(0);confD[ROLL].setMultiplier(0.1);confD[ROLL].setMax(255);

  confP[PITCH] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_PITCH",0,xParam+40,yParam+40,30,14));confP[PITCH].setDecimalPrecision(0);confP[PITCH].setMultiplier(0.1);confP[PITCH].setMax(255);
  confI[PITCH] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_PITCH",0,xParam+75,yParam+40,45,14));confI[PITCH].setDecimalPrecision(3);confI[PITCH].setMultiplier(0);confI[PITCH].setMax(0.255);
  confD[PITCH] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_PITCH",0,xParam+125,yParam+40,30,14));confD[PITCH].setDecimalPrecision(0);confD[PITCH].setMultiplier(0.1);confD[PITCH].setMax(255);

  confP[YAW] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_YAW",0,xParam+40,yParam+60,30,14));confP[YAW].setDecimalPrecision(0);confP[YAW].setMultiplier(0.1);confP[YAW].setMax(255);
  confI[YAW] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_YAW",0,xParam+75,yParam+60,45,14));confI[YAW].setDecimalPrecision(3);confI[YAW].setMultiplier(0);confI[YAW].setMax(0.255);
  confD[YAW] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_YAW",0,xParam+125,yParam+60,30,14));confD[YAW].setDecimalPrecision(0);confD[YAW].setMultiplier(0.1);confD[YAW].setMax(255);

  confP[ALT] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_ALT",0,xParam+40,yParam+97,30,14));confP[ALT].setDecimalPrecision(0);confP[ALT].setMultiplier(0.1);confP[ALT].setMax(255);
  confI[ALT] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_ALT",0,xParam+75,yParam+97,45,14));confI[ALT].setDecimalPrecision(3);confI[ALT].setMultiplier(0);confI[ALT].setMax(0.255);
  confD[ALT] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_ALT",0,xParam+125,yParam+97,30,14));confD[ALT].setDecimalPrecision(0);confD[ALT].setMultiplier(0.1);confD[ALT].setMax(255);

  confP[VEL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP_VEL",0,xParam+40,yParam+115,30,14));confP[VEL].setDecimalPrecision(0);confP[VEL].setMultiplier(0.1);confP[VEL].setMax(255);
  confI[VEL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI_VEL",0,xParam+75,yParam+115,45,14));confI[VEL].setDecimalPrecision(3);confI[VEL].setMultiplier(0.001);confI[VEL].setMax(0.255);
  confD[VEL] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD_VEL",0,xParam+125,yParam+115,30,14));confD[VEL].setDecimalPrecision(0);confD[VEL].setMultiplier(0.1);confD[VEL].setMax(255);confD[VEL].hide();
  
  AccRollTrim = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("ACCROLLTRIM",0,xParam+150,yParam+134,35,12));AccRollTrim.setDecimalPrecision(0);AccRollTrim.setMultiplier(0.1);AccRollTrim.setMax(50);
  AccPitchTrim = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("ACCPITCHTRIM",0,xParam+85,yParam+134,35,12));AccPitchTrim.setDecimalPrecision(0);AccPitchTrim.setMultiplier(0.1);AccPitchTrim.setMax(50);
  AccRollTrim.setColorBackground(grey_);AccRollTrim.setMin(-50);AccRollTrim.setDirection(Controller.HORIZONTAL);
  AccPitchTrim.setColorBackground(grey_);AccPitchTrim.setMin(-50);AccPitchTrim.setDirection(Controller.HORIZONTAL);

  for(int i=0;i<5;i++) {confP[i].setColorBackground(grey_);confP[i].setMin(0);confP[i].setDirection(Controller.HORIZONTAL);}
  for(int i=0;i<5;i++) {confI[i].setColorBackground(grey_);confI[i].setMin(0);confI[i].setDirection(Controller.HORIZONTAL);}
  for(int i=0;i<5;i++) {confD[i].setColorBackground(grey_);confD[i].setMin(0);confD[i].setDirection(Controller.HORIZONTAL);}
  //for(int i=3;i<4;i++) {confP[i].setColorBackground(grey_);confP[i].setMin(0);confP[i].setDirection(Controller.HORIZONTAL);}
  //for(int i=3;i<4;i++) {confI[i].setColorBackground(grey_);confI[i].setMin(0);confI[i].setDirection(Controller.HORIZONTAL);}
  //for(int i=3;i<4;i++) {confD[i].setColorBackground(grey_);confD[i].setMin(0);confD[i].setDirection(Controller.HORIZONTAL);}

  //rollPitchRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("rollPitchRate",0,xParam+160,yParam+30,40,14));rollPitchRate.setDecimalPrecision(2);rollPitchRate.setMultiplier(0.01);
  rollPitchRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("rollPitchRate",0,xParam+160,yParam+30,30,14));rollPitchRate.setDecimalPrecision(0);rollPitchRate.setMultiplier(0.1);
  rollPitchRate.setDirection(Controller.HORIZONTAL);rollPitchRate.setMin(0);rollPitchRate.setMax(5);rollPitchRate.setColorBackground(grey_);rollPitchRate.hide();
  yawRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("yawRate",0,xParam+160,yParam+60,30,14));yawRate.setDecimalPrecision(0);yawRate.setMultiplier(0.1);
  //yawRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("yawRate",0,xParam+160,yParam+60,40,14));yawRate.setDecimalPrecision(2);yawRate.setMultiplier(0.01);
  yawRate.setDirection(Controller.HORIZONTAL);yawRate.setMin(0);yawRate.setMax(5);yawRate.setColorBackground(grey_);yawRate.hide();
  
  confRC_RATE = controlP5.addNumberbox("RC RATE",1,xParam+223,yParam+109,42,14);confRC_RATE.setDecimalPrecision(2);confRC_RATE.setMultiplier(0.02);confRC_RATE.setLabel("");
  confRC_RATE.setDirection(Controller.HORIZONTAL);confRC_RATE.setMin(0);confRC_RATE.setMax(5);confRC_RATE.setColorBackground(grey_);confRC_RATE.hide();
  confRC_EXPO = controlP5.addNumberbox("RC EXPO",0,xParam+305,yParam+109,42,14);confRC_EXPO.setDecimalPrecision(2);confRC_EXPO.setMultiplier(0.01);confRC_EXPO.setLabel("");
  confRC_EXPO.setDirection(Controller.HORIZONTAL);confRC_EXPO.setMin(0);confRC_EXPO.setMax(0.99);confRC_EXPO.setColorBackground(grey_);confRC_EXPO.hide();

  buttonREAD =        controlP5.addButton("READ",1,xParam+10,yParam+187,39,16);buttonREAD.setColorBackground(red_);
  buttonWRITE =       controlP5.addButton("WRITE",1,xParam+128,yParam+187,46,16);buttonWRITE.setColorBackground(red_);
  buttonDEFAULTS =    controlP5.addButton("DEFAULTS",1,xParam+55,yParam+187,67,16);buttonDEFAULTS.setColorBackground(red_);
  buttonSTICKS =       controlP5.addButton("STICKS",1,xParam+214,yParam+187,49,16);buttonSTICKS.setColorBackground(red_);
  buttonCALIBRATE_ACC = controlP5.addButton("ACC",1,xParam+269,yParam+187,31,16);buttonCALIBRATE_ACC.setColorBackground(red_);
  buttonCALIBRATE_MAG = controlP5.addButton("GYRO",1,xParam+306,yParam+187,39,16);buttonCALIBRATE_MAG.setColorBackground(red_);
  buttonKK =          controlP5.addButton("KK",1,xParam+183,yParam+187,25,16);buttonKK.setColorBackground(red_);
  buttonLVAMODE =     controlP5.addButton("LED",1,xo+50,y1-30,55,16);buttonLVAMODE.setColorBackground(red_);
  
  rcStickThrottleSlider = controlP5.addSlider("Thro",900,2100,1500,xRC,yRC+0,100,20);rcStickThrottleSlider.setDecimalPrecision(0);
  rcStickPitchSlider =    controlP5.addSlider("Elev",900,2100,1500,xRC,yRC+26,100,20);rcStickPitchSlider.setDecimalPrecision(0);
  rcStickRollSlider =     controlP5.addSlider("Ail",900,2100,1500,xRC,yRC+52,100,20);rcStickRollSlider.setDecimalPrecision(0);
  rcStickYawSlider  =     controlP5.addSlider("Rud",900,2100,1500,xRC,yRC+78,100,20);rcStickYawSlider.setDecimalPrecision(0);
  rcStickAUX1Slider =    controlP5.addSlider("AUX1",900,2100,1500,xRC,yRC+104,100,20);rcStickAUX1Slider.setDecimalPrecision(0);
  rcStickAUX2Slider =    controlP5.addSlider("AUX2",900,2100,1500,xRC,yRC+130,100,20);rcStickAUX2Slider.setDecimalPrecision(0);
  rcStickAUX3Slider =     controlP5.addSlider("AUX3",900,2100,1500,xRC,yRC+156,100,20);rcStickAUX3Slider.setDecimalPrecision(0);
  rcStickAUX4Slider  =     controlP5.addSlider("AUX4",900,2100,1500,xRC,yRC+182,100,20);rcStickAUX4Slider.setDecimalPrecision(0);
  
  motSliderV0  = controlP5.addSlider("motSliderV0",100,2000,1500,0,0,20,150);motSliderV0.setDecimalPrecision(0);motSliderV0.hide();
  motSliderV1  = controlP5.addSlider("motSliderV1",100,2000,1500,0,0,20,150);motSliderV1.setDecimalPrecision(0);motSliderV1.hide();
  motSliderV2  = controlP5.addSlider("motSliderV2",100,2000,1500,0,0,20,150);motSliderV2.setDecimalPrecision(0);motSliderV2.hide();
  motSliderV3  = controlP5.addSlider("motSliderV3",100,2000,1500,0,0,20,150);motSliderV3.setDecimalPrecision(0);motSliderV3.hide();
  motSliderV4  = controlP5.addSlider("motSliderV4",100,2000,1500,0,0,10,100);motSliderV4.setDecimalPrecision(0);motSliderV4.hide();
  motSliderV5  = controlP5.addSlider("motSliderV5",100,2000,1500,0,0,10,100);motSliderV5.setDecimalPrecision(0);motSliderV5.hide();

  servoSliderH1  = controlP5.addSlider("Servo0",1000,2000,1500,0,0,100,10);servoSliderH1.setDecimalPrecision(0);servoSliderH1.hide();
  servoSliderH2 = controlP5.addSlider("Servo1",1000,2000,1500,0,0,100,10);servoSliderH2.setDecimalPrecision(0);servoSliderH2.hide();
  servoSliderH3 = controlP5.addSlider("Servo2",1000,2000,1500,0,0,100,10);servoSliderH3.setDecimalPrecision(0);servoSliderH3.hide();
  servoSliderH4 = controlP5.addSlider("Servo3",1000,2000,1500,0,0,100,10);servoSliderH4.setDecimalPrecision(0);servoSliderH4.hide();
  servoSliderV0  = controlP5.addSlider("Servov0",1000,2000,1500,0,0,10,100);servoSliderV0.setDecimalPrecision(0);servoSliderV0.hide();
  servoSliderV1  = controlP5.addSlider("Servov1",1000,2000,1500,0,0,10,100);servoSliderV1.setDecimalPrecision(0);servoSliderV1.hide();
  servoSliderV2 = controlP5.addSlider("Servov2",1000,2000,1500,0,0,10,100);servoSliderV2.setDecimalPrecision(0);servoSliderV2.hide();

  scaleSlider = controlP5.addSlider("SCALE",0,10,1,xGraph+400,yGraph-25,150,20);
  
  confPowerTrigger  = controlP5.addNumberbox("",0,xo+50,y1-50,55,14);confPowerTrigger.setDecimalPrecision(2);confPowerTrigger.setMultiplier(0.01);confPowerTrigger.setLabel("");
  confPowerTrigger.setDirection(Controller.HORIZONTAL);confPowerTrigger.setMin(0);confPowerTrigger.setMax(20);confPowerTrigger.setColorBackground(grey_);

  batSlider = controlP5.addSlider("",0,20,1,xo+50,y1-68,55,15);batSlider.setDecimalPrecision(2);
   
  //confPowerTrigger = controlP5.addNumberbox("",0,xGraph+50,yGraph-29,40,14);confPowerTrigger.setDecimalPrecision(0);confPowerTrigger.setMultiplier(10);
 // confPowerTrigger.setDirection(Controller.HORIZONTAL);confPowerTrigger.setMin(0);confPowerTrigger.setMax(65535);confPowerTrigger.setColorBackground(red_);
}

void draw() {
  int i;
  float val,inter,a,b,h;
 
  background(60);
  textFont(font15);
  text("MultiWii config",5,25);text("for OpenAero", 5, 43);
  text("Cycle Time:",xGraph+220,yGraph-10);text(cycleTime,xGraph+320,yGraph-10);

  textFont(font12);
  text("Based on MultiWii",3,450);text("by Alexander Dubus",3,465);
  text("GUI V0.21 for OpenAero V1.13",605,465);
   
  textFont(font12);
  //text("Power:",xGraph-5,yGraph-30); text(pMeterSum,xGraph+50,yGraph-30);
  //text("pAlarm:",xGraph-5,yGraph-15);  //text(intPowerTrigger,xGraph+50,yGraph-15);
  //text("Volt:",xGraph-5,yGraph-2);  text(bytevbat/10.0,xGraph+50,yGraph-2);

  time1=millis();
  if (init_com==1) {                          // Time data request pulses
    if ((time1-time2)>50 && (graph_on==1)) {
  //  if ((time1-time2)>150 && graph_on==1) {
      g_serial.write('M');
      time2=time1;
    }
    if (((time1-time3)>10) && (KKEnable==true) && (graph_on==0)) { // Time calibration pulses
       g_serial.write('w');
      time3=time1;
    }
  }
  
  axSlider.setValue(ax);aySlider.setValue(ay);azSlider.setValue(az);
  gxSlider.setValue(gx);gySlider.setValue(gy);gzSlider.setValue(gz);
  //baroSlider.setValue(baro/10);
  //magSlider.setValue(mag);
  //magxSlider.setValue(magx);magySlider.setValue(magy);magzSlider.setValue(magz);
  //debug1Slider.setValue(debug1/10);debug2Slider.setValue(debug2);debug3Slider.setValue(debug3);debug4Slider.setValue(debug4);


  // Vbat
  batSlider.setValue(pMeterSum);


  // Pass motor values to sliders
  motSliderV0.setValue(mot[0]);motSliderV1.setValue(mot[1]);motSliderV2.setValue(mot[2]);
  motSliderV3.setValue(mot[3]);motSliderV4.setValue(mot[4]);motSliderV5.setValue(mot[5]);

  //servoSliderH1.setValue(servo0);servoSliderH2.setValue(servo1);
  servoSliderH1.setValue(mot[2]); // Aeroplane elevator
  servoSliderH2.setValue(mot[0]); // Aeroplane rudder
  servoSliderH3.setValue(mot[5]); // Throttle
  servoSliderH4.setValue(mot[1]); // Aeroplane 1 aileron
  servoSliderV0.setValue(mot[1]); // Aeroplane 1 aileron, Aeroplane 2 left, FW left
  servoSliderV1.setValue(mot[2]); // FW right
  servoSliderV2.setValue(mot[3]); // Aeroplane 2 right

  //rcStickThrottleSlider.setValue(rcThrottle);rcStickRollSlider.setValue(rcRoll);rcStickPitchSlider.setValue(rcPitch);rcStickYawSlider.setValue(rcYaw);
  //rcStickThrottleSlider.setValue(rcYaw);rcStickRollSlider.setValue(rcRoll);rcStickPitchSlider.setValue(rcPitch);rcStickYawSlider.setValue(rcThrottle);
  rcStickThrottleSlider.setValue(rcYaw);rcStickRollSlider.setValue(rcRoll);rcStickPitchSlider.setValue(rcPitch);rcStickYawSlider.setValue(rcThrottle); // Fixed Roll input
  rcStickAUX1Slider.setValue(rcAUX1);rcStickAUX2Slider.setValue(rcAUX2);rcStickAUX3Slider.setValue(rcAUX3);rcStickAUX4Slider.setValue(rcAUX4);

  stroke(255); 
  // Fudge AngX, AngY for now until the KK+ can do so itself
  angx = ax/7;
  angy = -ay/7;
  h = radians(5); // To offset weird tilt to left... 5
  // Debug
  
  a=radians(angx);
  if (angy<-90) {
    b=radians(-180 - angy);
  } else if (angy>90) {
    b=radians(+180 - angy);
  } else
    b=radians(angy);
 // h=radians(mag);

  float size = 30.0;

  pushMatrix();
  //camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2+35,yObj/2-40,0,0,1,0);
  camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2,yObj/2,0,0,1,0);
  translate(xObj,yObj);
  //translate(xObj+35,yObj+35);
  directionalLight(200,200,200, 0, 0, -1);
  rotateZ(h);
  rotateX(b);
  rotateY(a); // - 0.1
  stroke(150,255,150);
  strokeWeight(0);sphere(size/3);strokeWeight(3);
  line(0,0, 10,0,-size-5,10);line(0,-size-5,10,+size/4,-size/2,10); line(0,-size-5,10,-size/4,-size/2,10);
  stroke(255);
  
  //multiType = 13; // debug
  
  if (multiType == 1) { //TRI
    ellipse(-size, -size, size, size);
    ellipse(+size, -size, size, size);
    ellipse(0,  +size,size, size);
    line(-size,-size, 0,0);
    line(+size,-size, 0,0);  
    line(0,+size, 0,0);
    noLights();
    textFont(font12);
    text(" TRICOPTER", -40,-50);camera();popMatrix();
 
    motSliderV0.setPosition(xMot+50,yMot+15);motSliderV0.setHeight(100);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot-15);motSliderV1.setHeight(100);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot-15);motSliderV2.setHeight(100);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    servoSliderH1.setPosition(xMot,yMot+135);servoSliderH1.setCaptionLabel("SERVO");servoSliderH1.show(); 

    motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
    servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();

  } else if (multiType == 2) { //QUAD+
    ellipse(0,  -size,   size,size);
    ellipse(0,  +size,   size, size);
    ellipse(+size, 0,    size , size );
    ellipse(-size, 0,    size , size );
    line(-size,0, +size,0);
    line(0,-size, 0,+size);
    noLights();
    textFont(font12);
    text("QUADCOPTER +", -40,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+50,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot+35);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot+35);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    motSliderV3.setPosition(xMot+50,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT");motSliderV3.show();
    
    motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 3) { //QUAD X
    ellipse(-size,  -size, size, size);
    ellipse(+size,  -size, size, size);
    ellipse(-size,  +size, size, size);
    ellipse(+size,  +size, size, size);
    line(-size,-size, 0,0);
    line(+size,-size, 0,0);
    line(-size,+size, 0,0);
    line(+size,+size, 0,0);
    noLights();
    textFont(font12);
    text("QUADCOPTER X", -40,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+10,yMot-10);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("FRONT_L");motSliderV0.show();
    motSliderV1.setPosition(xMot+85,yMot-10);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+85,yMot+78);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("REAR_R");motSliderV2.show();
    motSliderV3.setPosition(xMot+10,yMot+78);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("REAR_L");motSliderV3.show(); 
    
    
    motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 4) { //BI
    ellipse(0-size,  0,   size, size);
    ellipse(0+size,  0,   size, size);
    line(0-size,0, 0,0);  
    line(0+size,0, 0,0);
    line(0,size*1.5, 0,0);
    noLights();
    textFont(font12);
    text("BICOPTER", -30,-20);camera();popMatrix();
   
    motSliderV0.setPosition(xMot,yMot+30);motSliderV0.setHeight(55);motSliderV0.setCaptionLabel("");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot+30);motSliderV1.setHeight(55);motSliderV1.setCaptionLabel("");motSliderV1.show();
    servoSliderH1.setPosition(xMot,yMot+100);servoSliderH1.setWidth(60);servoSliderH1.setCaptionLabel("");servoSliderH1.show();
    servoSliderH2.setPosition(xMot+80,yMot+100);servoSliderH2.setWidth(60);servoSliderH2.setCaptionLabel("");servoSliderH2.show();
  } else if (multiType == 5) { //GIMBAL
    noLights();
    textFont(font12);
    text("GIMBAL", -20,-10);camera();popMatrix();
  
    textFont(font12);
    text("GIMBAL", xMot,yMot+25);
 
    servoSliderH3.setPosition(xMot,yMot+75);servoSliderH3.setCaptionLabel("ROLL");servoSliderH3.show();
    servoSliderH2.setPosition(xMot,yMot+35);servoSliderH2.setCaptionLabel("PITCH");servoSliderH2.show();

    motSliderV0.hide();motSliderV1.hide();motSliderV2.hide();motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 6) { //Y6
    ellipse(-size,-size,size,size);ellipse(size,-size,size,size);ellipse(0,-2+size,size,size);
    translate(0,0,7);
    ellipse(-5-size,-5-size,size,size);ellipse(5+size,-5-size,size,size);ellipse(0,3+size,size,size);
    line(-size,-size,0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    noLights();
    textFont(font12);
    text("TRICOPTER Y6", -40,-55);camera();popMatrix();

    motSliderV0.setPosition(xMot+50,yMot+23);motSliderV0.setHeight(50);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot-18);motSliderV1.setHeight(50);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot-18);motSliderV2.setHeight(50);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    motSliderV3.setPosition(xMot+50,yMot+87);motSliderV3.setHeight(50);motSliderV3.setCaptionLabel("U_REAR");motSliderV3.show();
    motSliderV4.setPosition(xMot+100,yMot+48);motSliderV4.setHeight(50);motSliderV4.setCaptionLabel("U_RIGHT");motSliderV4.show();
    motSliderV5.setPosition(xMot,yMot+48);motSliderV5.setHeight(50);motSliderV5.setCaptionLabel("U_LEFT");motSliderV5.show();

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 7) { //HEX6
    ellipse(-size,-0.55*size,size,size);ellipse(size,-0.55*size,size,size);ellipse(-size,+0.55*size,size,size);
    ellipse(size,+0.55*size,size,size);ellipse(0,-size,size,size);ellipse(0,+size,size,size);
    line(-size,-0.55*size,0,0);line(size,-0.55*size,0,0);line(-size,+0.55*size,0,0);
    line(size,+0.55*size,0,0);line(0,+size,0,0);  line(0,-size,0,0);
    noLights();
    textFont(font12);
    text("HEXACOPTER", -40,-50);camera();popMatrix();

    motSliderV0.setPosition(xMot+90,yMot+65);motSliderV0.setHeight(50);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-5);motSliderV1.setHeight(50);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+5,yMot+65);motSliderV2.setHeight(50);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+5,yMot-5);motSliderV3.setHeight(50);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    motSliderV4.setPosition(xMot+50,yMot-20);motSliderV4.setHeight(50);motSliderV4.setCaptionLabel("FRONT");motSliderV4.show(); 
    motSliderV5.setPosition(xMot+50,yMot+90);motSliderV5.setHeight(50);motSliderV5.setCaptionLabel("REAR");motSliderV5.show(); 

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();

  } else if (multiType == 8) { //FLYING_WING
    line(0,0, 1.8*size,size);line(1.8*size,size,1.8*size,size-30);  line(1.8*size,size-30,0,-1.5*size);
    line(0,0, -1.8*size,+size);line(-1.8*size,size,-1.8*size,+size-30);    line(-1.8*size,size-30,0,-1.5*size);
    noLights();
    textFont(font12);
    text("FLYING WING", -40,-50);camera();popMatrix();

    servoSliderV0.setPosition(xMot+10,yMot+10);servoSliderV0.setCaptionLabel("LEFT");servoSliderV0.show(); 
    servoSliderV1.setPosition(xMot+90,yMot+10);servoSliderV1.setCaptionLabel("RIGHT");servoSliderV1.show(); 

    motSliderV0.hide();motSliderV1.hide();motSliderV2.hide();motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
  } else if (multiType == 9) { //Y4
    ellipse(-size,  -size, size, size);
    ellipse(+size,  -size, size, size);
    ellipse(0,  +size, size+2, size+2);
    line(-size,-size, 0,0);
    line(+size,-size, 0,0);
    line(0,+size, 0,0);
    translate(0,0,7);
    ellipse(0,  +size, size, size);

    noLights();
    textFont(font12);
    text("Y4", -5,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+80,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR_1");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-15);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+30,yMot+75);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("REAR_2");motSliderV2.show();
    motSliderV3.setPosition(xMot+10,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    
    motSliderV4.hide();motSliderV5.hide();
    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 10) { //HEX6 X
    ellipse(-0.55*size,-size,size,size);ellipse(-0.55*size,size,size,size);ellipse(+0.55*size,-size,size,size);
    ellipse(+0.55*size,size,size,size);ellipse(-size,0,size,size);ellipse(+size,0,size,size);
    line(-0.55*size,-size,0,0);line(-0.55*size,size,0,0);line(+0.55*size,-size,0,0);
    line(+0.55*size,size,0,0);line(+size,0,0,0);  line(-size,0,0,0);
    noLights();
    textFont(font12);
    text("HEXACOPTER X", -45,-50);camera();popMatrix();

    motSliderV0.setPosition(xMot+80,yMot+90);motSliderV0.setHeight(45);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+80,yMot-20);motSliderV1.setHeight(45);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+25,yMot+90);motSliderV2.setHeight(45);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+25,yMot-20);motSliderV3.setHeight(45);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    motSliderV4.setPosition(xMot+90,yMot+35);motSliderV4.setHeight(45);motSliderV4.setCaptionLabel("RIGHT");motSliderV4.show(); 
    motSliderV5.setPosition(xMot+5,yMot+35);motSliderV5.setHeight(45);motSliderV5.setCaptionLabel("LEFT");motSliderV5.show(); 

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 11) { //OCTOX8
    noLights();
    textFont(font12);
    text("OCTOCOPTER X8", -45,-50);camera();popMatrix();

    servoSliderH1.hide();servoSliderH2.hide();servoSliderH3.hide();servoSliderH4.hide();
    servoSliderV0.hide();servoSliderV1.hide();servoSliderV2.hide();
  } else if (multiType == 12) { //Aeroplane
      // Wings
    line(1.8*size,size-50,0,-0.7*size);      //Right LE
    line(0,0, 1.8*size,size-40);             //Right TE
    line(1.8*size,size-40,1.8*size,size-50); //Right tip
    line(-1.8*size,size-50,0,-0.7*size);     //Left LE
    line(0,0, -1.8*size,size-40);            //Left TE
    line(-1.8*size,size-50,-1.8*size,size-40);//Left tip  
    // Tailplane
    line(0,20, size,size);                //Right LE
    line(size,size+10,0,size+7);          //Right TE
    line(size,size+10,size,size);         //Right tip
    line(0,20, -size,size);               //Left LE
    line(-size,size+10,0,size+7);         //Left TE
    line(-size,size,-size,size+10);       //Left tip  
    //Center and rudder
    line(0,-0.9*size,0,size+15);
    
    noLights();
    textFont(font12);
    text("AEROPLANE1", -40,-50);camera();popMatrix();

    servoSliderH1.setPosition(xMot+10,yMot+10);servoSliderH1.setCaptionLabel("ELE");servoSliderH1.show(); // Elevator
    servoSliderH2.setPosition(xMot+10,yMot+30);servoSliderH2.setCaptionLabel("RUD");servoSliderH2.show(); // Rudder
    servoSliderH3.setPosition(xMot+10,yMot+50);servoSliderH3.setCaptionLabel("THR");servoSliderH3.show(); // Throttle
    servoSliderV0.setPosition(xMot+50,yMot+70);servoSliderV0.setCaptionLabel("AIL");servoSliderV0.show(); // Left
    servoSliderV1.hide();
    servoSliderV2.hide();
    servoSliderH4.hide();
    motSliderV0.hide();motSliderV1.hide();motSliderV2.hide();motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
     
  } else if (multiType == 13) { // Flaperon Aeroplane
      // Wings
    line(1.8*size,size-50,0,-0.7*size);      //Right LE
    line(0,0, 1.8*size,size-40);             //Right TE
    line(1.8*size,size-40,1.8*size,size-50); //Right tip
    line(-1.8*size,size-50,0,-0.7*size);     //Left LE
    line(0,0, -1.8*size,size-40);            //Left TE
    line(-1.8*size,size-50,-1.8*size,size-40);//Left tip  
    // Tailplane
    line(0,20, size,size);                //Right LE
    line(size,size+10,0,size+7);          //Right TE
    line(size,size+10,size,size);         //Right tip
    line(0,20, -size,size);               //Left LE
    line(-size,size+10,0,size+7);         //Left TE
    line(-size,size,-size,size+10);       //Left tip  
    //Center and rudder
    line(0,-0.9*size,0,size+15);
    
    noLights();
    textFont(font12);
    text("AEROPLANE2", -40,-50);camera();popMatrix();

    servoSliderH1.setPosition(xMot+10,yMot+10);servoSliderH1.setCaptionLabel("ELE");servoSliderH1.show(); // Elevator
    servoSliderH2.setPosition(xMot+10,yMot+30);servoSliderH2.setCaptionLabel("RUD");servoSliderH2.show(); // Rudder
    servoSliderH3.setPosition(xMot+10,yMot+50);servoSliderH3.setCaptionLabel("THR");servoSliderH3.show(); // Throttle
    servoSliderV0.setPosition(xMot+10,yMot+70);servoSliderV0.setCaptionLabel("LEFT");servoSliderV0.show(); // Left
    servoSliderV2.setPosition(xMot+90,yMot+70);servoSliderV2.setCaptionLabel("RIGHT");servoSliderV2.show(); // Right
    servoSliderH4.hide();
    servoSliderV1.hide();
    motSliderV0.hide();motSliderV1.hide();motSliderV2.hide();motSliderV3.hide();motSliderV4.hide();motSliderV5.hide();
  }
  else {
    noLights();
    camera();
    popMatrix();
  }
  
  pushMatrix();
  translate(xObj,yObj-250);
  //translate(xObj+50,yObj-165);
  textFont(font15);text("ROLL", -90, 5);
  rotate(a);
  line(-30,0,+30,0);line(0,0,0,-10);
  popMatrix();
  
  pushMatrix();
  //translate(xObj+50,yObj-100);
  translate(xObj,yObj-190);
  textFont(font15);text("PITCH", -90, 5);
  rotate(b);
  line(-30,0,30,0);line(+30,0,30-size/2 ,size/3);  line(+30,0,30-size/2 ,-size/3);  
  popMatrix();
 
  pushMatrix();
  translate(xObj-70,yObj-220);
  //translate(xObj-20,yObj-133);
  size=15;
  strokeWeight(1.5);
  fill(0);stroke(0);
  ellipse(0,  0,   2*size+7, 2*size+7);
  stroke(255);
  float head= mag*PI/180;
  rotate(head);
  line(0,size, 0,-size);   
  line(0,-size, -5 ,-size+10);  
  line(0,-size, +5 ,-size+10);  
  popMatrix();
    
  strokeWeight(1);
  fill(255, 255, 255);
  g_graph.drawGraphBox();
  
  strokeWeight(1.5);
  stroke(255, 0, 0);
  if (axGraph) g_graph.drawLine(accROLL, -1000, +1000);
  stroke(0, 255, 0);
  if (ayGraph) g_graph.drawLine(accPITCH, -1000, +1000);
  stroke(0, 0, 255);
  if (azGraph) {
   if (scaleSlider.value()<2) g_graph.drawLine(accYAW, -1000, +1000);
   else g_graph.drawLine(accYAW, 200*scaleSlider.value()-1000,200*scaleSlider.value()+500);
  }
  
  float BaroMin = (baroData.getMinVal() + baroData.getRange() / 2) - 10;
  float BaroMax = (baroData.getMinVal() + baroData.getRange() / 2) + 10;
  
  stroke(200, 200, 0);  if (gxGraph)   g_graph.drawLine(gyroROLL, -300, +300);
  stroke(0, 255, 255);  if (gyGraph)   g_graph.drawLine(gyroPITCH, -300, +300);
  stroke(255, 0, 255);  if (gzGraph)   g_graph.drawLine(gyroYAW, -300, +300);
  stroke(125, 125, 125);if (baroGraph) g_graph.drawLine(baroData, BaroMin, BaroMax);

  stroke(225, 225, 125);if (magGraph)  g_graph.drawLine(magData, -370, +370);
  stroke(50, 100, 150); if (magxGraph)  g_graph.drawLine(magxData, -500, +500);
  stroke(100, 50, 150); if (magyGraph)  g_graph.drawLine(magyData, -500, +500);
  stroke(150, 100, 50); if (magzGraph)  g_graph.drawLine(magzData, -500, +500);

  stroke(0, 0, 0);
  //if (debug1Graph)  g_graph.drawLine(debug1Data, BaroMin, BaroMax);
  //if (debug2Graph)  g_graph.drawLine(debug2Data, -5000, +5000);
  //if (debug3Graph)  g_graph.drawLine(debug3Data, -5000, +5000);
  //if (debug4Graph)  g_graph.drawLine(debug4Data, -5000, +5000);

  fill(0, 0, 0); // debug

  strokeWeight(3);
  stroke(0);
  rectMode(CORNERS);
  rect(xMot,yMot-20, xMot+140, yMot+193);
  rect(xRC-5,yRC-5, xRC+140, yRC+208);
  rect(xParam,yParam, xParam+355, yParam+213);
 
  // New box for param buttons 
  fill(60, 60, 60); // debug
  rect(xParam+5,yParam+165, xParam+350, yParam+208);
  
  int xSens       = xParam + 220; // Expo chart origins
//  int ySens       = yParam + 65;
  int ySens       = yParam + 15;
  /*
  stroke(255);
  a=min(confRC_RATE.value(),1);
  b=confRC_EXPO.value();
  strokeWeight(1);
  line(xSens,ySens,xSens,ySens+80); // Vert
  line(xSens,ySens+80,xSens+120,ySens+80); // Horiz
  strokeWeight(3);
  stroke(30,120,30); // Colour?
  for(i=0;i<120;i++) {  //length
    inter = 10*i;
//    val = a*inter*(1-b+inter*inter*b/490000);
    val = a*inter*(1-b+inter*inter*b/1350000);
    point(xSens+1+i,ySens+38+(70-val/10)*4/7);
//        point(xSens+1+i,ySens+38+(70-val/10)*4/7);
  }
  if (confRC_RATE.value()>1) { // Make tiny circle
    stroke(220,100,100);
    ellipse(xSens+120, ySens+10, 7, 7);
  }
  */
  fill(255);
  textFont(font15);    
  text("P",xParam+48,yParam+15);text("I",xParam+93,yParam+15);
  text("D",xParam+132,yParam+15);
  textFont(font12);
  //text("RATE",xParam+191,yParam+122);
  //text("EXPO",xParam+270,yParam+122);
  text("----------- CALIBRATE -----------",xParam+185,yParam+182);
  text("---- FLIGHT PARAMETERS ----",xParam+12,yParam+182);

  //text("RATE",xParam+165,yParam+15);
  text("ROLL",xParam+3,yParam+32);text("PITCH",xParam+3,yParam+52);text("YAW",xParam+3,yParam+72);
  text("AUTOLEVEL",xParam+3,yParam+92);
  text("GYRO",xParam+3,yParam+109);
  text("ACC.",xParam+3,yParam+126);
  text("ACC.TRIM",xParam+3,yParam+145);
  text("L",xParam+140,yParam+145);
  text("R",xParam+190,yParam+145);
  text("D",xParam+75,yParam+145);
  text("U",xParam+125,yParam+145);
}

void ACC_ROLL(boolean theFlag) {axGraph = theFlag;}
void ACC_PITCH(boolean theFlag) {ayGraph = theFlag;}
void ACC_Z(boolean theFlag) {azGraph = theFlag;}
void GYRO_ROLL(boolean theFlag) {gxGraph = theFlag;}
void GYRO_PITCH(boolean theFlag) {gyGraph = theFlag;}
void GYRO_YAW(boolean theFlag) {gzGraph = theFlag;}
void BARO(boolean theFlag) {baroGraph = theFlag;}
void HEAD(boolean theFlag) {magGraph = theFlag;}
void MAGX(boolean theFlag) {magxGraph = theFlag;}
void MAGY(boolean theFlag) {magyGraph = theFlag;}
void MAGZ(boolean theFlag) {magzGraph = theFlag;}
void DEBUG1(boolean theFlag) {debug1Graph = theFlag;}
void DEBUG2(boolean theFlag) {debug2Graph = theFlag;}
void DEBUG3(boolean theFlag) {debug3Graph = theFlag;}
void DEBUG4(boolean theFlag) {debug4Graph = theFlag;}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup())
    if (theEvent.name()=="portComList") InitSerial(theEvent.group().value()); // initialize the serial port selected
}

public void bSTART() {
  if(graphEnable == false) {return;}
  graph_on=1;
  readEnable = true;calibrateEnable = true;
  buttonREAD.setColorBackground(green_);
  buttonCALIBRATE_ACC.setColorBackground(green_);
  buttonCALIBRATE_MAG.setColorBackground(green_);
  buttonSTICKS.setColorBackground(green_);
  buttonDEFAULTS.setColorBackground(green_);
  buttonWRITE.setColorBackground(red_);
  g_serial.clear();
}

public void bSTOP() {
  graph_on=0;
  if (writeEnable) buttonWRITE.setColorBackground(green_);
}

public void READ() {
  if(readEnable == false) {return;}
  for(int i=0;i<4;i++) {
    confP[i].setValue(byteP[i]);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);
  }
  confP[4].setValue(byteP[4]);confI[4].setValue(byteI[4]/1000.0);
  
  confRC_RATE.setValue(byteRC_RATE/50.0);
  confRC_EXPO.setValue(byteRC_EXPO/100.0);
  rollPitchRate.setValue(byteRollPitchRate);
  yawRate.setValue(byteYawRate);

  if (LVAMode == 1) {
    buttonLVAMODE.setCaptionLabel("BUZZER");
    LVAButtonMode = true;
  }
  if (LVAMode == 0) {
    buttonLVAMODE.setCaptionLabel(" LED");
    LVAButtonMode = false;
  }
  buttonLVAMODE.setColorBackground(green_);

  for(int i=0;i<5;i++) {confP[i].setColorBackground(green_);}
  for(int i=4;i<5;i++) {confI[i].setColorBackground(green_);}
  for(int i=0;i<4;i++) {confD[i].setColorBackground(green_);}
  
  confRC_RATE.setColorBackground(green_);
  confRC_EXPO.setColorBackground(green_);
  rollPitchRate.setColorBackground(green_);
  yawRate.setColorBackground(green_);

  confPowerTrigger.setColorBackground(green_);
  confPowerTrigger.setValue(intPowerTrigger/100.0);

  AccRollTrim.setValue(byteAccRollTrim-127); // Offset trims -127 to 128 (127 = 0)
  AccPitchTrim.setValue(byteAccPitchTrim-127); // Offset trims -127 to 128
  AccRollTrim.setColorBackground(green_);
  AccPitchTrim.setColorBackground(green_);
  
  writeEnable = true;  
  if (graph_on == 0) buttonWRITE.setColorBackground(green_);
}

public void WRITE() {
  if((writeEnable == false) || (graph_on == 1)) {return;} //Debug - Don't allow writing while graphing

  for(int i=0;i<4;i++) {byteP[i] = (round(confP[i].value()));}
  for(int i=0;i<4;i++) {byteI[i] = (round(confI[i].value()*1000));}
  for(int i=0;i<4;i++) {byteD[i] = (round(confD[i].value()));}
  byteP[4] = (round(confP[4].value()));
  byteI[4] = (round(confI[4].value()*1000));
 
  byteRC_RATE = (round(confRC_RATE.value()*50));
  byteRC_EXPO = (round(confRC_EXPO.value()*100));
  byteRollPitchRate = (round(rollPitchRate.value()));
  byteYawRate = (round(yawRate.value()));
  intPowerTrigger = (round(confPowerTrigger.value()*100)); //LVA
  //intPowerTrigger = (round(confPowerTrigger.value())); //LVA
  //byteAccRollTrim = AccRollTrim.value();
  //byteAccPitchTrim = AccPitchTrim.value();
  byteAccRollTrim = (round(AccRollTrim.value()+127));
  byteAccPitchTrim = (round(AccPitchTrim.value()+127));
  
  if (LVAButtonMode == true) // Buzzer
  {
    present = (present | 0x10);  // Set LVA bit (Buzzer)
  } 
  else // LED
  {
    present = (present & 0xEF); // Clear LVA bit (LED)
  }

  int[] s = new int[32];
  
  int p = 0;
   s[p++] = 'W'; //1 write to Eeprom @ KK
   for(int i=0;i<3;i++) {s[p++] = byteP[i];  s[p++] = byteI[i];  s[p++] =  byteD[i];} //10
   s[p++] = byteP[3];  s[p++] = byteI[3]; //12 glevel
   s[p++] = byteP[4];  s[p++] = byteI[4]; //14 alevel
   s[p++] = byteRC_RATE;
   s[p++] = byteRC_EXPO; //16
   s[p++] = byteRollPitchRate; 
   s[p++] = byteYawRate;
   s[p++] = intPowerTrigger ; // LSB
   s[p++] = intPowerTrigger >>8 &0xff; //20
   s[p++] = present;  //21
   s[p++] = byteAccRollTrim;//22
   s[p++] = byteAccPitchTrim;//23
   for(int i =0;i<23;i++)    g_serial.write(char(s[i]));
}

public void ACC() {
  if(calibrateEnable == false) {return;}
  g_serial.write('S'); // acc Sensor calibration request
}
public void GYRO() {
  if(calibrateEnable == false) {return;}
  g_serial.write('E'); // gyro Sensor calibration request
}
public void STICKS() {
  g_serial.write('C'); // Stick calibration request
}
public void DEFAULTS() {
  g_serial.write('D'); // Load default parameters request
}
public void KK() { // Start/stop KK calibration mode
  if (KKEnable == true)
  {
    KKEnable = false;
    buttonKK.setColorBackground(green_);
  }
  else
  {
    KKEnable = true;
    buttonKK.setColorBackground(orange_);
  }
}

public void LED() {  // Switch LVA mode (buzzer = 1)
  if (writeEnable)
  { 
    if (LVAButtonMode == false) // if was LED
    {
      LVAButtonMode = true;
      buttonLVAMODE.setCaptionLabel("BUZZER");
      buttonLVAMODE.setColorBackground(green_);
    }
    else if (LVAButtonMode == true) // if was buzzer
    {
      LVAButtonMode = false;
      buttonLVAMODE.setCaptionLabel(" LED");
      buttonLVAMODE.setColorBackground(green_);
    }
  }
}

// initialize the serial port selected in the listBox
void InitSerial(float portValue) {
  String portPos = Serial.list()[int(portValue)];
  txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
  //g_serial = new Serial(this, portPos, 115200);
  //g_serial = new Serial(this, portPos, 76800);
  g_serial = new Serial(this, portPos, 19200);
  //g_serial = new Serial(this, portPos, 9600); 
  init_com=1;
  buttonSTART.setColorBackground(green_);buttonSTOP.setColorBackground(green_);commListbox.setColorBackground(green_);
  buttonKK.setColorBackground(green_);
  graphEnable = true;
  g_serial.buffer(frame_size+1);
}

int p;
byte[] inBuf = new byte[frame_size];

int read16() {return (inBuf[p++]&0xff) + (inBuf[p++]<<8);}
int read8()  {return inBuf[p++]&0xff;}

void serialEvent(Serial p) { 
  processSerialData(); 
}

void processSerialData() {
  //int present=0,mode=0;

  if (g_serial.read() == 'M') {
    g_serial.readBytes(inBuf);
    if (inBuf[frame_size-1] == 'M') {  // Multiwii @ arduino send all data to GUI
      p=0;
     // New data format - start
      read8(); //version                                                              //1
      ax = read16();ay = read16();az = read16();                                      //7
      gx = read16();gy = read16();gz = read16();                                      //13
      for(int i=0;i<6;i++) mot[i] = read16();                                         //25
      rcRoll = read16();rcPitch = read16();rcYaw = read16();rcThrottle = read16();    //33
      rcAUX1 = read16();rcAUX2 = read16();rcAUX3 = read16();rcAUX4 = read16();        //41
      present = read8();                                                              //
      cycleTime = read16();                                                           //
      multiType = read8();                                                            //45
      for(int i=0;i<3;i++) {byteP[i] = read8();byteI[i] = read8();byteD[i] = read8();}//54
      byteP[3] = read8();byteI[3] = read8();
      byteP[4] = read8();byteI[4] = read8();                                          //58
      byteRC_RATE = read8();                                                          //
      byteRC_EXPO = read8();                                                          //60
      byteRollPitchRate = read8();                                                    //
      byteYawRate = read8();                                                          //62
      pMeterSum = read16();                                                           //
      intPowerTrigger = read16();                                                     //66
      byteAccRollTrim = read8();                                                   
      //
      byteAccPitchTrim = read8();                                                     //68
      // New data format - end

      
      if ((present&1) >0) AutoLevelON = 1; else  AutoLevelON = 0;
      if ((present&2) >0) NormalMode = 1; else  NormalMode = 0;
      if ((present&4) >0) AcroMode = 1; else  AcroMode = 0;
      if ((present&8) >0) StabilityMode = 1; else  StabilityMode = 0;
      if ((present&16) >0) LVAMode = 1; else  LVAMode = 0;

      if (AutoLevelON>0) {buttonNunchuk.setColorBackground(green_);} else {buttonNunchuk.setColorBackground(red_);}
      if (NormalMode>0) {buttonI2cAcc.setColorBackground(green_);} else {buttonI2cAcc.setColorBackground(red_);}
      if (AcroMode>0) 
      {
        buttonI2cBaro.setColorBackground(green_);
        buttonI2cBaro.setLabel(" EEPROM settings");
      } 
      else 
      {
        buttonI2cBaro.setColorBackground(red_);
        buttonI2cBaro.setLabel("     Pot settings");
      }
      
      if (StabilityMode>0) {buttonI2cMagneto.setColorBackground(green_);} else {buttonI2cMagneto.setColorBackground(red_);}
  
      accROLL.addVal(ax);accPITCH.addVal(ay);accYAW.addVal(az);
      gyroROLL.addVal(gx);gyroPITCH.addVal(gy);gyroYAW.addVal(gz);
      
      pMeterSum = pMeterSum/100;
    } //if (inBuf[frame_size-1] == 'M')
  } //if (g_serial.read() == 'M')
  else
  {
    g_serial.readStringUntil('M');
  }
} //void processSerialData()


//********************************************************
//********************************************************
//********************************************************

class cDataArray {
  float[] m_data;
  int m_maxSize;
  int m_startIndex = 0;
  int m_endIndex = 0;
  int m_curSize;
  
  cDataArray(int maxSize){
    m_maxSize = maxSize;
    m_data = new float[maxSize];
  }
  void addVal(float val) {
    m_data[m_endIndex] = val;
    m_endIndex = (m_endIndex+1)%m_maxSize;
    if (m_curSize == m_maxSize) {
      m_startIndex = (m_startIndex+1)%m_maxSize;
    } else {
      m_curSize++;
    }
  }
  float getVal(int index) {return m_data[(m_startIndex+index)%m_maxSize];}
  int getCurSize(){return m_curSize;}
  int getMaxSize() {return m_maxSize;}
  float getMaxVal() {
    float res = 0.0;
    for(int i=0; i<m_curSize-1; i++) 
      if ((m_data[i] > res) || (i==0)) res = m_data[i];
    return res;
  }
  float getMinVal() {
    float res = 0.0;
    for(int i=0; i<m_curSize-1; i++) 
      if ((m_data[i] < res) || (i==0)) res = m_data[i];
    return res;
  }
  float getRange() {return getMaxVal() - getMinVal();}
}

// This class takes the data and helps graph it
class cGraph {
  float m_gWidth, m_gHeight;
  float m_gLeft, m_gBottom, m_gRight, m_gTop;
  
  cGraph(float x, float y, float w, float h) {
    m_gWidth     = w;
    m_gHeight    = h;
    m_gLeft      = x;
    m_gBottom    = y;
    m_gRight     = x + w;
    m_gTop       = y + h;
  }
  
  void drawGraphBox() {
    stroke(0, 0, 0);
    rectMode(CORNERS);
    rect(m_gLeft, m_gBottom, m_gRight, m_gTop);
  }
  
  void drawLine(cDataArray data, float minRange, float maxRange) {
    float graphMultX = m_gWidth/data.getMaxSize();
    float graphMultY = m_gHeight/(maxRange-minRange);
    
    for(int i=0; i<data.getCurSize()-1; ++i) {
      float x0 = i*graphMultX+m_gLeft;
      float y0 = m_gTop-(((data.getVal(i)-(maxRange+minRange)/2)*scaleSlider.value()+(maxRange-minRange)/2)*graphMultY);
      float x1 = (i+1)*graphMultX+m_gLeft;
      float y1 = m_gTop-(((data.getVal(i+1)-(maxRange+minRange)/2 )*scaleSlider.value()+(maxRange-minRange)/2)*graphMultY);
      line(x0, y0, x1, y1);
    }
  }
}
