

long lastBeatTime = 0; //Time at which the last beat occurred
float beatsPerMinute;

//long interval = 30000; //30000 this is how long we capture hr data

bool captureHR(uint32_t startTime) {
  if (millis() - startTime > interval) {
#ifdef DEBUG
    Serial.println("HR capture done");
#endif
    return(false);
  }

  redSample = particleSensor.getRed();
  irSample = particleSensor.getIR();
  sumSample = redSample + irSample;

  sumSample = irSample + redSample; // + greenSample; * seems to be the best
  sumEMA_S = (EMA_a*sumSample) + ((1-EMA_a)*sumEMA_S);  //run the EMA
  sumHighpass = sumSample - sumEMA_S;                           //calculate the high-pass signal

  while (SimbleeBLE.radioActive) {
    ;
  }

  if (checkForPulse(sumHighpass) == true)
  {
    analogWrite(RED, 200);
    //We sensed a beat!
    if(firstBeat){
      firstBeat = false;
      lastBeatTime = millis();
      return(true);
    }
    long delta = millis() - lastBeatTime;
    lastBeatTime = millis();

    beatsPerMinute = 60 / (delta / 1000.0); // get instantaneous BPM

    if (beatsPerMinute < 200 && beatsPerMinute > 50)
    {
        beat = beatsPerMinute;
        arrayBeats[beatCounter] = (uint8_t)beat;
        beatCounter++;
        if(beatCounter > 255){ beatCounter = 0; }
    #ifdef DEBUG
        Serial.print("instant BPM: ");
        Serial.println(beat);
    #endif
    }
  }
  digitalWrite(RED, HIGH);


}




bool checkForPulse(int sample){

boolean beatDetected = false;
  if (sample < T){
    if(!rising){
      T = sample;
    }else{
      rising = true;
      edge = true;
    }
  }
  if (sample > P){
    if(rising){
      P = sample;
    }else{
      rising = false;
      edge = true;
    }
  }

  if(edge){
    edge = false;
    amp = P-T;
    if(rising){
      T = P;
    }else{
      T = P;
    }
    if(amp>50 && amp<6000){
      beatDetected = true;
    }
  }
  return beatDetected;
  }




/*
bool captureHR(uint32_t startTime) {
  if (millis() - startTime > interval) {
#ifdef DEBUG
    Serial.println("HR capture done");
#endif
    return(false);
  }

//  long greenSample = particleSensor.getGreen();
//  long redSample = particleSensor.getRed();
  long irSample = particleSensor.getIR();

  while (SimbleeBLE.radioActive) {
    ;
  }
//  if (checkForBeat(greemnSample) == true)
//  if (checkForBeat(redSample) == true)
  if (checkForBeat(irSample) == true)
  {
    analogWrite(RED, 200);
    //We sensed a beat!
    if(firstBeat){
      firstBeat = false;
      lastBeatTime = millis();
      return(true);
    }
    long delta = millis() - lastBeatTime;
    lastBeatTime = millis();

    beatsPerMinute = 60 / (delta / 1000.0); // get instantaneous BPM

    if (beatsPerMinute < 200 && beatsPerMinute > 50)
    {
        beat = beatsPerMinute;
        // if(bConnected) {
        //         SimbleeBLE.send((uint8_t)beatAvg);
        // }
    }
  }
  digitalWrite(RED, HIGH);
  //Here is where you could send all the beats if you wanted

  if (beat != lastBeat) {
    arrayBeats[beatCounter] = (uint8_t)beat;
    beatCounter++;
    if(beatCounter > 255){ beatCounter = 0; }
#ifdef DEBUG
    Serial.print("instant BPM: ");
    Serial.println(beat);
#endif
    // if(bConnected) {
    //         SimbleeBLE.sendInt(beatAvg);
    // }

    lastBeat = beat;
  }
  return(true);
}



/*
 *        OLD AVERAGING CODE
 *
 *    rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      if(rateSpot == RATE_SIZE-1){
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
 *
 */
