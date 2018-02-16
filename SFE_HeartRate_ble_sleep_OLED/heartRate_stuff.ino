



bool captureHR(uint32_t startTime) {
  if (millis() - startTime > interval) {
#ifdef DEBUG
    Serial.println("HR capture done");
#endif

    return (false);
  }
  //analogWrite(GRN,200);
  long irValue = particleSensor.getGreen();
  //digitalWrite(GRN,HIGH);
  //digitalWrite(RED,LOW);
  while (SimbleeBLE.radioActive) {
    ;
  }
  if (checkForBeat(irValue) == true)
  {
    analogWrite(BLU, 200);
    //We sensed a beat!
    //digitalWrite(GRN,LOW);
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      // if(bConnected) {
      //         SimbleeBLE.send((uint8_t)beatAvg);
      // }
    }
  }
  digitalWrite(BLU, HIGH);
  //digitalWrite(GRN,HIGH);
  //Here is where you could send all the beats if you wanted

  if (beatAvg != lastBeatAvg) {
    aveBeatsAve[aveCounter] = (uint8_t)beatAvg;
    aveCounter++;
#ifdef DEBUG
    Serial.print("Beat Ave: ");
    Serial.println(beatAvg);
#endif
    // if(bConnected) {
    //         SimbleeBLE.sendInt(beatAvg);
    // }

  }
  lastBeatAvg = beatAvg;
  return (true);
}
