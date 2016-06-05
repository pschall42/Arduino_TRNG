#include <LiquidCrystal.h>
#include <Wire.h>


#define REDLITE 7
#define GREENLITE 6
#define BLUELITE 5

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

int brightness = 255;
float avg;

const int NUM_PINS = 5;
const int NUM_LATCHES = 8;
const int UNSTABLE_CIRCUIT = 3;
const int GEIGER_COUNTER = 5;
const int SR_LATCHES = 4;
int input_pins[NUM_PINS];
word data[NUM_PINS];
word same, diff, outputL, outputH, temp, sum, mask;
int controller;
long output;
// timing for the geiger counter
unsigned long start, prev, current, delta, bips, iterations;
float bps;


void setup() {
  // initialize variables
  iterations = 0;
  start = micros();
  prev = start;
  current = start;
  bips = 0;
  bps = 0.0;
  sum = 0x0000;
  outputL = 0x0000;
  outputH = 0x0000;
  // set pin modes for digital pins 22-29
  for (int i = 0; i < NUM_LATCHES; i++)
  {
    pinMode(i+22, INPUT);
  }
  // set pin mode for digital pin 20 (geiger counter, needs interrupt)
  pinMode(20, INPUT);
  attachInterrupt(digitalPinToInterrupt(20), geigerInterrupt, RISING);

  // set up the input pins
  for (int i = 0; i < NUM_PINS; i++)
  {
    input_pins[i] = i;
  }
  
  lcd.begin(16, 2);
  pinMode(REDLITE, OUTPUT);
  pinMode(GREENLITE, OUTPUT);
  pinMode(BLUELITE, OUTPUT);
 
  brightness = 100;
  setBacklight(150, 0, 150);
  avg = 300.0;

  Serial.begin(9600);
}

void loop() {
  // sparkfun parts sometimes have arduino code on how to use
  // arduino noise filter arduino code is on drive
  iterations++;
  
  // read 10 bit analog inputs
  for (int i = 0; i < NUM_PINS; i++)
  {
    data[i] = word(analogRead(input_pins[i]));
    if (i == UNSTABLE_CIRCUIT)
    {
      // read unstable circuit
      getCircuitNoise();
    }
    else if (i == SR_LATCHES)
    {
      // read SR latches
      getSRLatchesIn();
    }
    else if (i == GEIGER_COUNTER)
    {
      // read geiger counter
      getGeigerCounterIn(output % 3);
    }
  }

  // use 0, 1, 2, or 3 for bit mixing, anything else for the first input
  procOut(0);
  mixBits(3);
  procOut(1);
  output = wordsToLong(outputH, outputL);

  // write outputH and outputL to digital out pins
  printToScreen(output);
  //Serial.print("\nOutput:\n");
  //if (iterations % 10000)
  //{
    Serial.print(output);
    Serial.print("\n");
  //}
  //delay(1000);
}

// geiger counter interrupt routine
void geigerInterrupt()
{
  prev = current;
  current = micros();
  bips++;
}

void getCircuitNoise()
{
  data[UNSTABLE_CIRCUIT] = analogRead(UNSTABLE_CIRCUIT);
  //byte adc_byte = adc_value >> 2;
}

void getSRLatchesIn()
{
  word sr_data = 0x0000;
  avg /= float(iterations * NUM_LATCHES);
  for(int i = 0; i < NUM_LATCHES; i++)
  {
    //int b = digitalRead(i+22);
    int b = analogRead(i+SR_LATCHES);
    /*Serial.print("\n\nInput ");
    Serial.print(i+3);
    Serial.print(" = ");
    Serial.print(b);
    Serial.print("\n\n");*/
    if (b >= 200)
    {
      sr_data = sr_data ^ 0x0001;
    }
    sr_data = sr_data << 1;
    avg += (float)b;
  }
  /*Serial.print("\n");
  Serial.print(sr_data);
  Serial.print("\n");*/
  data[SR_LATCHES] = sr_data;
}

void getGeigerCounterIn(int mode)
{
  word upper, lower;
  float tmp;
  long* p = (long*) &tmp; // used to bit shift a float
  delta = current - prev; // for use if using time between bips
  bps = (float)bips / (float)(current - start);
  tmp = bps;
  switch (mode)
  {
    // gets 10 bit upper and lower values based on bps
    case 0:
      upper = word(*p >> 14);
      lower = word((*p << 14) >> 14);
      break;
    // gets 10 bit upper and lower values based on time between bips
    case 1:
      upper = word(delta >> 14);
      lower = word((delta << 14) >> 14);
      break;
    // gets 10 bit upper and lower values based on both bps and time between bips
    case 2:
      upper = word(*p >> 14) ^ word(delta >> 14);
      lower = word((*p << 14) >> 14) ^ word((delta << 14) >> 14);
      break;
  }
  data[GEIGER_COUNTER] = (upper ^ lower);
  // reset counts if high enough, since bps won't vary much when sample time is high
  if(current - start > (10000000 + ((unsigned)output >> 6))) // current - start > (10 + (0-67.1)) sec
  {
    bips = 0;
    start = micros();
    prev = current;
    current = start;
  }
}

void mixBits(int mode)
{
  // find similar bits
  same = data[0];
  same = ~(same ^ data[0]);
  for (int i = 1; i < NUM_PINS; i++)
  {
    same = same & ~(same ^ data[i]); // bitwise xnor to filter which bits are similar between them
  }
  diff = ~same;
  switch (mode)
  {
    // returns the similar bits XORed with the XOR sum of the different bits
    case 0:
      // bit mixing
      //outputL = 0x0000;
      //outputH = 0x0000;
      outputL = data[0] & same; // gets the similar bits
      // get the xor sum of the remaining bits
      for(int i = 0; i < NUM_PINS; i++)
      {
        temp = diff & data[i];
        sum = temp ^ sum;
      }
      outputL = outputL ^ sum; // combine the two
      
      // get lower order bits of the lower 16 bits
      outputL = outputL & 0x00FF;
      // get the xnor sum of the inputs right shifted a random amount
      sum = 0x0000;
      for(int i = 0; i < NUM_PINS; i++)
      {
        sum = ~(data[i] >> (int(outputL) % 3) ^ sum);
      }
      sum = sum << 8;
      // get higher order bits of the lower 16 bits
      outputL = outputL ^ sum;

      // get the xnor sum of the inputs right shifted a random amount
      sum = 0x0000;
      for(int i = 0; i < NUM_PINS; i++)
      {
        sum = ~(data[i] ^ sum);
      }
      // get lower order bits of the upper 16 bits
      outputH = ~sum & 0x00FF;

      // get the xor not sum of the inputs right shifted a random amount
      sum = 0x0000;
      for(int i = 0; i < NUM_PINS; i++)
      {
        sum = (data[i] >> (int(outputL) % 3) ^ ~sum);
      }
      sum = sum << 8;
      // get higher order bits of the upper 16 bits
      outputH = outputH ^ sum;
      break;
      // gets the lower order bits of the inputs, based on a control input, and concatenates them
    case 1:
      // finds which input controls the order the inputs are concatenated
      controller = (int)(outputH ^ outputL);
      controller = controller % NUM_PINS;
      //outputL = 0x0000;
      //outputH = 0x0000;

      mask = 0x0007;
      for (int i = 0; i < 4; i++)
      {
        if (i < 3)
        {
          // picks an input to get the lower order bits for
          temp = data[controller] & mask;
          temp = temp >> 3 * i;
          // reads the lower order bits and shifts them to the right position for concatenation
          temp = data[int(temp) % (NUM_PINS)] & 0x00FF;
          temp = temp << 8 * (i % 2);
          // concatenates the input pieces with the current output
          if (i < 2)
          {
            outputL = outputL ^ temp;
          }
          else
          {
            outputH = outputH ^ temp;
          }
          // shift the mask left 3 places to determine the next input
          mask = mask << 3;
        }
        else
        {
          //mask = mask >> 3;
          // shift the mask a random amount between 1 and 9 places
          mask = mask >> ((long((outputH ^ outputL)) % 9) + 1); 
          temp = data[controller % (NUM_PINS)] & mask;
          temp = temp >> (9 - ((long((outputH ^ outputL)) % 9) + 1)); // shifts bits back so an input can be obtained
          
          // reads the lower order bits and shifts them to the right position for concatenation
          temp = data[int(temp) % (NUM_PINS)] & 0x00FF;
          temp = temp << 8;
          
          // concatenates the input pieces with the current output
          outputH = outputH ^ temp;
        }
      }
      break;
    // use similar and different bits in conjunction to generate an output
    case 2:
      // finds which input controls the order the inputs are concatenated
      controller = (int)(same);
      //outputL = 0x0000;
      //outputH = 0x0000;

      for (int i = 0; i < 4; i++)
      {
        // get the input
        controller = controller % NUM_PINS;
        temp = data[controller] & diff & 0x00FF;
        temp = temp << 8 * (i % 2);
        // concatenates the input pieces with the current output
        if (i < 2)
        {
          outputL = outputL ^ temp;
          // change the input used
          controller = int(outputL);
        }
        else
        {
          outputH = outputH ^ temp;
          // change the input used
          controller = int(outputH ^ outputL);
        }
      }
      break;
    // takes the xor sum of the outputs of the other algorithms and uses that as the output
    case 3:
      word upper, lower;
      for(int i = 0; i < 3; i++)
      {
        mixBits(i);
        if (i == 0)
        {
          upper = outputH;
          lower = outputL;
        }
        else
        {
          upper = upper ^ outputH;
          lower = lower ^ outputL;
        }
      }
      outputH = upper;
      outputL = lower;
      break;
    default:
      outputH = 0x0000;
      outputL = data[GEIGER_COUNTER];
      break;
  }
}

// preprocessing and postprocessing of outputs
void procOut(int mode)
{
  word xorSum, notSum, temp;
  int test;
  long total;
  xorSum = 0x0000;
  notSum = xorSum;
  for (int i = 0; i < NUM_PINS; i++)
  {
    xorSum = xorSum ^ data[i];
    notSum = (~notSum) ^ data[i];
  }
  temp = xorSum ^ ((xorSum ^ notSum) & outputL); // merge the two according to a mask of outputL
  total = (long)int(temp);
  test = int(temp) % 8; // approximately 50% chance to change outputL
  switch (temp)
  {
    // invert
    case 0:
      outputL = ~outputL;
      break;
    // reverse the bits, see http://graphics.stanford.edu/~seander/bithacks.html#ReverseParallel
    case 1:
      outputL = ((outputL >> 1) & 0x5555) | ((outputL & 0x5555) << 1); // switch adjacent bits
      outputL = ((outputL >> 2) & 0x3333) | ((outputL & 0x3333) << 2); // switch groups of 2 bits
      outputL = ((outputL >> 4) & 0x0F0F) | ((outputL & 0x0F0F) << 4); // switch groups of 4 bits
      outputL = ((outputL >> 8) & 0x00FF) | ((outputL & 0x00FF) << 8); // switch bytes
      break;
    // swap the bytes
    case 2:
      outputL = ((outputL >> 8) & 0x00FF) | ((outputL & 0x00FF) << 8);
      break;
    // xor with the merged value
    case 3:
      outputL = outputL ^ temp;
      break;
    // set outputL to 0, only works if mode == 0 (preprocessing), do nothing otherwise
    case 4:
      if (mode == 0)
      {
        outputL = 0x0000;
      }
      break;
    // do nothing to the output
    default:
      break;
  }
  temp = xorSum ^ ((xorSum ^ notSum) & outputH); // merge the two according to a mask of outputH
  total += (long)int(temp);
  test = int(temp) % 8; // approximately 50% chance to change outputH
  switch (temp)
  {
    // invert
    case 0:
      outputH = ~outputH;
      break;
    // reverse the bits, see http://graphics.stanford.edu/~seander/bithacks.html#ReverseParallel
    case 1:
      outputH = ((outputH >> 1) & 0x5555) | ((outputH & 0x5555) << 1); // switch adjacent bits
      outputH = ((outputH >> 2) & 0x3333) | ((outputH & 0x3333) << 2); // switch groups of 2 bits
      outputH = ((outputH >> 4) & 0x0F0F) | ((outputH & 0x0F0F) << 4); // switch groups of 4 bits
      outputH = ((outputH >> 8) & 0x00FF) | ((outputH & 0x00FF) << 8); // switch bytes
      break;
    // swap the bytes
    case 2:
      outputH = ((outputH >> 8) & 0x00FF) | ((outputH & 0x00FF) << 8);
      break;
    // xor with the merged value
    case 3:
      outputH = outputH ^ temp;
      break;
    // set outputH to 0, only works if mode == 0 (preprocessing), do nothing otherwise
    case 4:
      if (mode == 0)
      {
        outputL = 0x0000;
      }
      break;
    // do nothing to the output
    default:
      break;
  }
  // switches upper and lower order words at a rate of 1/7
  test = (int)(total % 7);
  if (test == 0)
  {
    temp = outputH;
    outputH = outputL;
    outputL = temp;
  }
}

// returns a 32 bit integer generated from the upper and lower order 16 bit words
long wordsToLong (word upper, word lower)
{
  unsigned long res;
  long result;
  res = (long)int(upper);
  // effectively shifts the bits to bit positions 31-16
  res *= 65536;
  res += (long)int(lower);
  result = res;
  return result;
}

void printToScreen(long num)
{
  // insert code here
  lcd.setCursor(0, 1);
  lcd.clear();
  lcd.print(num);
}

void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
  // normalize the red LED - its brighter than the rest!
  //r = map(r, 0, 255, 0, 100);
  //g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);
 
  Serial.print("R = "); Serial.print(r, DEC);
  Serial.print(" G = "); Serial.print(g, DEC);
  Serial.print(" B = "); Serial.println(b, DEC);
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}

