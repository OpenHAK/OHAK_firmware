


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
