// Constants for anemometer filter
const float expWeightingFactor = 0.07;
const int medFactor = 50;

float expValue = 0;
bool expFirstValueFlag = true;
float exponentialFilter(float reading){
   //first value
   if (expFirstValueFlag == true){
      expValue = reading;
      expFirstValueFlag = false;
   }else{
      //deal with vaue wrapping
      if (reading < 90 && expValue > 270){
         expValue -= 360;
      }else if (reading > 270 && expValue < 90){
         expValue += 360;
      }
      expValue = expWeightingFactor*reading + (1 - expWeightingFactor)*expValue;
   }
    
   return fmod(expValue + 360, 360);
}

float medValue[medFactor+1];
float medValueSorted[medFactor+1];
bool medFirstValueFlag = true;
float medianFilter(float reading){
   // initialize arrays with first reading
   if (medFirstValueFlag){
      for (int i = 0; i < medFactor; i++){
         medValue[i] = reading;
         medValueSorted[i] = reading;
      }
        
      medFirstValueFlag = false;
   }

   // wrap data if necessary
   if (reading < 90 && medValue[0] > 270){
      for (int i = 0; i < medFactor; i++){
         medValue[i] = medValue[i] - 360;
         medValueSorted[i] = medValueSorted[i] - 360;
      }
   }else if (reading > 270 && medValue[0] < 90){
      for (int i = 0; i < medFactor; i++){
         medValue[i] = medValue[i] + 360;
         medValueSorted[i] = medValueSorted[i] + 360;
      }
   }

   //remove last value
   for (int i = 0; i < medFactor-1; i++){
      if (medValueSorted[i] >= medValue[medFactor-1]){
         medValueSorted[i] = medValueSorted[i+1];
      }
   }

   //insert new value
   int i = medFactor-1;
   while (i > 0 && medValueSorted[i-1] > reading){
      medValueSorted[i] = medValueSorted[i-1];
      i--;
   }
   medValueSorted[i] = reading;

   //shift chronological values
   for (int i = medFactor-1; i > 0; i--){
      medValue[i] = medValue[i-1];
   }
   medValue[0] = reading;
    
   return fmod(medValueSorted[int(medFactor/2)] +360, 360);
}
