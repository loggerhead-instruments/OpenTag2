#define CMD(a,b) ( a + (b << 8))
#define TRUE 1
#define FALSE 0

int ProcCmd(char *pCmd)
{
  short *pCV;
  short n;
  long lv1;
  char s[22];
  unsigned int tday;
  unsigned int tmonth;
  unsigned int tyear;
  unsigned int thour;
  unsigned int tmin;
  unsigned int tsec;

  pCV = (short*)pCmd;

  n = strlen(pCmd);
  if(n<2) return TRUE;

  switch(*pCV)
  {                     
    
  // Accelerometer full scale
    case ('A' + ('G'<<8)):
    {
      sscanf(&pCmd[3],"%d",&lv1);
      accel_scale = lv1;
      break;
    }

    // Set Real Time Clock
    case ('T' + ('M'<<8)):
    {
         //set time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear, &tmonth, &tday, &thour, &tmin, &tsec);
         rtc.setTime(thour, tmin, tsec);
         rtc.setDate(tday, tmonth, tyear);
         SerialUSB.print("Year:"); SerialUSB.println(tyear);
         SerialUSB.print("month:"); SerialUSB.println(tmonth);
         SerialUSB.print("day:"); SerialUSB.println(tday);
         SerialUSB.print("hour:"); SerialUSB.println(thour);
         SerialUSB.print("minute:"); SerialUSB.println(tmin);
         SerialUSB.print("second:"); SerialUSB.println(tsec);
         SerialUSB.println("Time set from script");
         break;
     }
/*
    case ('B' + ('W'<<8)):
    {
         //set time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         tmElements_t NewTime;
         NewTime.Second = tsec;
         NewTime.Minute = tmin;
         NewTime.Hour = thour;
         NewTime.Day = tday;
         NewTime.Month = tmonth;
         NewTime.Year = tyear + 2000 - 1970;
         burnTime = makeTime(NewTime); // makeTime is offset from 1970
         burnFlag = 1;
         SerialUSBprint("Burn Time:");
         SerialUSBprintln(burnTime);
         break;
      }

    // Burn Minutes (burn set number of minutes after start)
    case ('B' + ('M'<<8)):
    {
         sscanf(&pCmd[3],"%d",&lv1);
         burnMinutes = lv1;
         burnFlag = 2;
         break;
      }
      */
      case ('R' + ('D'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        recDur = lv1;
        SerialUSB.print("Rec dur:");
        SerialUSB.println(recDur);
        break;
      }
      
      case ('R' + ('I'<<8)):
      {
        sscanf(&pCmd[3],"%d",&lv1);
        recInt = lv1;
        SerialUSB.print("Rec int:");
        SerialUSB.println(recInt);
        break;
      } 

      /*
      case ('S' + ('R'<<8)):
      {
        //start time
         sscanf(&pCmd[3],"%d-%d-%d %d:%d:%d",&tyear,&tmonth,&tday,&thour,&tmin,&tsec);
         tmElements_t NewTime;
         NewTime.Second = tsec;
         NewTime.Minute = tmin;
         NewTime.Hour = thour;
         NewTime.Day = tday;
         NewTime.Month = tmonth;
         NewTime.Year = tyear-2000;
         startTime = makeTime(NewTime);
         SerialUSBprint("Start Record Set: ");
         SerialUSBprintln(startTime);
         break;
      } 
      */
  } 
  return TRUE;
}

boolean loadScript()
{
  char s[30];
  char c;
  short i;

  File file;
  unsigned long TM_byte;
  int comment_TM = 0;

  // Read card setup.txt file to set date and time, recording interval
 file=sd.open("setup.txt");
 if(file)
 {
   do{
        i = 0;
        s[i] = 0;
        do{
            c = file.read();
            if(c!='\r') s[i++] = c;
            if(c=='T') 
            {
              TM_byte = file.position() - 1;
              comment_TM = 1;
            }
            if(i>29) break;
          }while(c!='\n');
          s[--i] = 0;
          if(s[0] != '/' && i>1)
          {
            ProcCmd(s);
          }
      }while(file.available());
      file.close();  
      
      // comment out TM line if it exists
      if (comment_TM)
      {
        SerialUSB.print("Comment TM ");
        SerialUSB.println(TM_byte);
        file = sd.open("setup.txt", FILE_WRITE);
        file.seek(TM_byte);
        file.print("//");
        file.close();
      }
      
  }
  else
  {   
    SerialUSB.println("setup.txt not opened");

   // display.println("no setup file");
    return 0;
  }
 return 1;  
}
