/*
  SIMPLE HIGH PASS FILTER
*/


float RClow, RChigh;
float dt;
float aLow, aHigh;


void initFilter(){
  RClow = 1.0/(CUTTOFF_LOW*2*PI);
  RChigh = 1.0/(CUTTOFF_HIGH*2*PI);
  dt = 1.0/sampleRate;
  aLow = RClow/(RClow+dt);
  aHigh = RChigh/(RChigh+dt);
  for(int i=0; i<NUM_SAMPLES; i++){
    LPfilterInputRED[i] = 0.0; LPfilterOutputRED[i] = 0.0;
    LPfilterInputIR[i] = 0.0; LPfilterOutputIR[i] = 0.0;
    HPfilterInputRED[i] = 0.0; HPfilterOutputRED[i] = 0.0;
    HPfilterInputIR[i] = 0.0; HPfilterOutputIR[i] = 0.0;
  }
}

void runFilters(int rVal, int irVal){
  filterLP(rVal, irVal);
//  filterHP(rVal, irVal);
}

void filterLP(int rVal, int irVal){
  LPfilterOutputRED[0] = LPfilterInputRED[0] = float(rVal); 
  LPfilterOutputIR[0] = LPfilterInputIR[0] = float(irVal);
  for(int i=1; i<NUM_SAMPLES; i++){
    LPfilterOutputRED[i] = LPfilterOutputRED[i-1]+(aLow*(LPfilterInputRED[i] - LPfilterOutputRED[i-1]));
    LPfilterOutputIR[i] = LPfilterOutputIR[i-1]+(aLow*(LPfilterInputIR[i] - LPfilterOutputIR[i-1]));
    
    LPfilterInputRED[i] = LPfilterInputRED[i-1];  // move the input array along
    LPfilterInputIR[i] = LPfilterInputIR[i-1];  // move the input array along
  }
  filterHP(LPfilterOutputRED[NUM_SAMPLES-1], LPfilterOutputIR[NUM_SAMPLES-1]);
}

void filterHP(int rVal, int irVal){
  HPfilterOutputRED[0] = HPfilterInputRED[0] = float(rVal); 
  HPfilterOutputIR[0] = HPfilterInputIR[0] = float(irVal);
  for(int i=1; i<NUM_SAMPLES; i++){
    HPfilterOutputRED[i] = aHigh*(HPfilterOutputRED[i-1]+HPfilterInputRED[i]-HPfilterInputRED[i-1]);
    HPfilterOutputIR[i] = aHigh*(HPfilterOutputIR[i-1]+HPfilterInputIR[i]-HPfilterInputIR[i-1]);

    HPfilterInputRED[i] = HPfilterInputRED[i-1];
    HPfilterInputIR[i] = HPfilterInputIR[i-1];
  }
}
