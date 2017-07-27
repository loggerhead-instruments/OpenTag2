/*  Watchdog timer support for Arduino Zero
 *  by Richard Hole  December 19, 2015
 */

/*  Functions included:
 *  
 *  setupWDT(uint8_t period);
 *              initializes WDT in NORMAL mode
 *              initializes WDT period, 
 *                  valid values: 0x0 to 0xB (0-11)
 *                  see 17.18.2 and Table 17-5 in Atmel SAM D21G Datasheet
 *              enables WDT timer
 *  
 *  resetWDT();
 *              resets the WDT timer to prevent automatic system reset on timeout
 *  
 *  systemReset();
 *              forces an immediate system reset
 *              MUST first have WDT enabled 
 */
static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

//============= resetWDT ===============
void resetWdt() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  WDT->CLEAR.reg= 0xA5; // reset the WDT
  WDTsync(); 
}

//============= systemReset =============
void systemReset() {
  // use the WDT watchdog timer to force a system reset.
  // WDT MUST be running for this to work
  WDT->CLEAR.reg= 0x00; // system reset via WDT
  WDTsync(); 
}

//============= setupWDT ================
void setupWDT( uint8_t period) {
  // initialize the WDT watchdog timer

  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required

  WDT->CONFIG.reg = min(period,11); // see Table 17-5 Timeout Period (valid values 0-11)

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync(); 
}

