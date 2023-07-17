/*  ESP32 ADC1 I2S _alternating_ channel DMA sampler for Ardumower compatible border signal (2 signals).
   For testing it can generate a synchronous BASEFREQ square wave on PWMPIN and a 1/4 square wave on PWMPIN2.

   It fetches 12bit NUMSAMPLES from 2 perimeter channels in the background to DMA buffer, then these get copied to foreground.
   Note: To minimise channel switching artefacts 2 extra samples are taken which then are discarded.
   In result buffer, the top 4 bits are the channel number.
   Additionally, 4 more ADC1 channels are read "normally" in as single values e.g. for current measurements.
   Note: The ESP32 ADC DMA is a bit strange on top of I2S interface with lots of artefacts...
   This sketch does work with ESP32 Arduino core 2.09 !!!
   Includes WIFI and telnet server.
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "robot.h"
#include <limits.h>
#include "adcman.h"
#include "config.h"
#include "buzzer.h"
#include "flashmem.h"

#define telnetDebug(x) telnet.println(x)
			   
// start includes
// WIFI

#define PWMPIN 18 // for test signal
#define PWMPIN2 25 // for test signal
#define BASEFREQ (9615) // base signal frequency
#define ADCFREQ (BASEFREQ*4) // 4*oversampling

#define I2S_NUM I2S_NUM_0 // I2S channel to use
#define NUMSAMPLES (192) // 4 times oversampling of 2 24 bit signals
#define SHOWSAMPLES (192) // how much to show in plotter interface

#define firstChannel ADC1_CHANNEL_0 // ADC input for channel 0: 4=GPIO32(PWMfast), 5=GPIO33(free), 0=GPIO36(per), 3=GPIO39(per), 6=GPIO34(cur), 7=GPIO35(PWMslow)
#define secondChannel ADC1_CHANNEL_3 // ADC input for channel 1 

adc1_channel_t  ADC = firstChannel;
adc1_channel_t  ADC1 = secondChannel;

//#define DEBUGSIGNAL // if we want a debug PWM signal

int chanFlag = 0; // auto toggle perimeter channel to scan
volatile int leftCurVal = 555;
volatile int rightCurVal = 666;
volatile int supplyVal = 777;
volatile int x = 888;

extern Mower robot; 

static uint16_t i2s_raw_buff[NUMSAMPLES + 2]; // our 12 bit work buffer
int16_t ofs[2] = {2048, 2048}; // ADC zero offsets - after 5k6/10k divider and dynamically adapted
int16_t ADCMin[2]; // ADC min sample values
int16_t ADCMax[2]; // ADC max sample values
int16_t ADCAvg[2]; // ADC avg sample values
int16_t center0, center1;
extern int8_t sigcode_diff[]; // the signal sent by perimeter sender

static unsigned long volatile msstart, msend;
static int64_t volatile end_time, start_time;

unsigned long nextPeri, nextPeriShow;
unsigned long nextLoopTime = 3000;

bool doShow = false;
bool plot = false;

static esp_adc_cal_characteristics_t adc1_chars;

static float calibrate_adc(uint8_t chan, uint16_t value) // to optionally correct for ESP32 ADC flaws
{
  // y = ax + b
  return 0.0008100147275 * value + 0.1519145803;
}

void i2s_adc_init() // set up ADC DMA
{
  // the basic I2S aDC configuration
  i2s_config_t i2s_config;
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  i2s_config.sample_rate = ADCFREQ;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT; // 16bit to get 2*dma_buf_len.
  i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.dma_buf_count = 2; // dma_desc_num. Min 2, Max 128.
  i2s_config.dma_buf_len = 192 + 2; // dma_frame_num. Min 8, Max 1024. Size * 2, in i2s.c
  i2s_config.use_apll = false;           // Can't be used with internal ADC.
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.fixed_mclk = 0; // fi2s_clk can be set manually, but i2s calculations will screw it anyway.
  i2s_config.tx_desc_auto_clear = false;

  Serial.printf("Attempting to setup I2S ADC with sampling frequency %d Hz\n", ADCFREQ);
  if (ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)) {
    Serial.printf("Error installing I2S. Halt!");
    while (1);
  }
  if (ESP_OK != i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0)) {
    Serial.printf("Error setting up ADC. Halt!");
    while (1);
  }
  if (ESP_OK != adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11)) {
    Serial.printf("Error setting up ADC attenuation. Halt!");
    while (1);
  }
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

  if (ESP_OK != i2s_adc_enable(I2S_NUM_0)) {
    Serial.printf("Error enabling ADC. Halt!");
    while (1);
  }
  Serial.printf("ESP32 I2S DMA ADC setup ok\n");
  // delay for I2S bug workaround
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

size_t i2s_run(int chan) // chan: which channel to read, prepare next
{
  size_t bytes_read = 0; // how much we got from DMA into i2s_raw_buff
  //memset(i2s_raw_buff,0x55,(NUMSAMPLES + 2) * 2);

  ESP_ERROR_CHECK(i2s_read(I2S_NUM_0, (void *)i2s_raw_buff, (NUMSAMPLES + 2) * 2, &bytes_read, portMAX_DELAY)); //(needs ca. 4ms)
  if (bytes_read != (NUMSAMPLES + 2) * 2)
  {
    Serial.printf("===== Sample count error ==================\n\n\n");
    while (1);
  }
  i2s_adc_disable(I2S_NUM_0);
  i2s_zero_dma_buffer(I2S_NUM_0);
  leftCurVal = adc1_get_raw(ADC1_CHANNEL_6); // curl
  rightCurVal = adc1_get_raw(ADC1_CHANNEL_7); // curr
  supplyVal = adc1_get_raw(ADC1_CHANNEL_4); // volt
  x = adc1_get_raw(ADC1_CHANNEL_5);

  if (chan & 1)
  {
    if (ESP_OK != i2s_set_adc_mode(ADC_UNIT_1, ADC)) {
      Serial.printf("Error setting up ADC. Halt!");
      while (1);
    }
  }
  else
  {
    if (ESP_OK != i2s_set_adc_mode(ADC_UNIT_1, ADC1)) {
      Serial.printf("Error setting up ADC. Halt!");
      while (1);
    }
  }
  if (ESP_OK != i2s_adc_enable(I2S_NUM_0)) {
    Serial.printf("Error enabling ADC. Halt!");
    while (1);
  }
  return bytes_read;
}

//set up PWM output for debugging
void ledc_init(void)
{
  ledcSetup(3, BASEFREQ, 8); // PWM, 8-bit resolution
  ledcAttachPin(PWMPIN, 3);
  ledcWriteTone(3, BASEFREQ);

  ledcSetup(4, BASEFREQ / 4, 8); // PWM, 8-bit resolution
  ledcAttachPin(PWMPIN2, 4);
  ledcWriteTone(4, BASEFREQ / 4);
						  
											   
									
							
							
								  
}

// generate a 8bit perimeter raw buffer with min/max from 12 bit samples
void postProcess2(int sample)
{
  extern int8_t sigcode_diff[]; // the template signal
  float avg = 0;
  int j = 0;

  ADCMax[sample] = -9999;
  ADCMin[sample] = 9999;
								   
 

  for (int i = 0; i < robot.perimeter.numSamples; i ++)
    i2s_raw_buff[i] = ~i2s_raw_buff[i]; // dma sampled data is inverted

  for (int i = 0; i < robot.perimeter.numSamples; i += 2) // copy over 12 bit DMA data to 8 bit raw buffers
  {
    int16_t value0, value1; // we need to swap samples and skip first extra samples - values are negated
    value0 = (min(SCHAR_MAX,  max(SCHAR_MIN, ((int16_t)(i2s_raw_buff[i + 2] & 0xfff) - ofs[sample] + 16) >> 4)));
    value1 = (min(SCHAR_MAX,  max(SCHAR_MIN, ((int16_t)(i2s_raw_buff[i + 2 + 1] & 0xfff) - ofs[sample] + 16) >> 4)));

    robot.perimeter.raw_buff[sample][j++] = value1;
    robot.perimeter.raw_buff[sample][j++] = value0;
							  
 

    /*robot.perimeter.raw_buff[1][j - 2] = value1 / 64;
      robot.perimeter.raw_buff[1][j - 1] = value0 / 64;*/
								  
 

    ADCMax[sample] = max(ADCMax[sample], value0);
    ADCMin[sample] = min(ADCMin[sample], value0);
    ADCMax[sample] = max(ADCMax[sample], value1);
    ADCMin[sample] = min(ADCMin[sample], value1);
    avg += ((float)value0) / ((float)robot.perimeter.numSamples);
    avg += ((float)value1) / ((float)robot.perimeter.numSamples);
  }

  center0 = ADCMin[sample] + (ADCMax[sample] - ADCMin[sample]) / 2.0;
  ADCAvg[sample] = avg;
  //printf("AVG:%d, %d, %d, %d, %d, %d\n", ADCAvg[0], 0, center0, 0, 0, 0);
  ofs[sample] += (ADCAvg[sample] * 16); // auto-correct offset
}

void adcsetup(void)
{
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
#ifdef DEBUGSIGNAL
  ledc_init();
  // alternatively ouput a fixed value for debugging
  //dac_output_enable(DAC_CHANNEL_1);
  //dac_output_voltage(DAC_CHANNEL_1, 100);
#endif
  i2s_adc_init();
  //printf("Initialization finished, output on pin %d\n", PWMPIN);
}

#define ADDR 0
#define MAGIC 1

#define ADC_SAMPLE_COUNT_MAX 2048  // warning usefull with 128bit sigcode but slow the compute on all the PCB and slow down the tracking
#define INVALID_CHANNEL 99

#define NO_CHANNEL 255

uint16_t dmaData[ADC_SAMPLE_COUNT_MAX];
ADCManager ADCMan;


ADCManager::ADCManager(){
  convCounter = 0;  
  chNext = 0;
  chCurr = INVALID_CHANNEL;    
  for (int i=0; i < ADC_CHANNEL_COUNT_MAX; i++){
    channels[i].sampleCount = 0;
    channels[i].zeroOfs =0;
    channels[i].value =0;
    channels[i].convComplete = false;
  }  
  sampleRate = SRATE_38462;   // sampling frequency 38462 Hz
}


void ADCManager::begin(){ 
adcsetup();   
}

void ADCManager::printInfo(){
  Console.println(F("---ADC---"));  
  Console.print(F("conversions="));
  Console.println(convCounter);
  Console.print(F("sampleRate="));
  switch (sampleRate){
    case SRATE_38462: Console.println(F("38462")); break;
    case SRATE_19231: Console.println(F("19231")); break;
    case SRATE_9615 : Console.println(F("9615")); break;
  }    
  for (int ch=0; ch < ADC_CHANNEL_COUNT_MAX; ch++){
    if (channels[ch].sampleCount != 0){
      Console.print(F("AD"));
      Console.print(ch);
      Console.print(F("\t"));    
      Console.print(F("sampleCount="));    
      Console.println(channels[ch].sampleCount);      
      Console.print(F("\t"));    
      Console.print(F("autoCalibrate="));    
      Console.println(channels[ch].autoCalibrate);      
    }
  }
  
}

int ADCManager::getConvCounter(){
  int res = convCounter;
  convCounter = 0;
  return res;
}

void ADCManager::setupChannel(byte pin, int samplecount, boolean autocalibrate){
  byte ch = pin-A0;
  pinMode(pin, INPUT);
  channels[ch].pin = pin; 
  channels[ch].autoCalibrate = autocalibrate;  
  channels[ch].convComplete = false;
  channels[ch].maxValue = 0;
  channels[ch].minValue = 0;
  setSampleCount(ch, samplecount);
}

void ADCManager::setSampleCount(byte ch, int samplecount){
  samplecount = min(samplecount, ADC_SAMPLE_COUNT_MAX);
  channels[ch].samples = (int8_t *)realloc(channels[ch].samples, samplecount);
  channels[ch].sampleCount = samplecount;  
}

boolean ADCManager::isConvComplete(byte pin){
  byte ch = pin-A0;
  return channels[ch].convComplete;
}


void ADCManager::restartConv(byte pin){
  byte ch = pin-A0;
  channels[ch].convComplete = false;
}

int8_t* ADCManager::getSamples(byte pin){
  byte ch = pin-A0;
  return channels[ch].samples;
}

int ADCManager::getSampleCount(byte pin){
  byte ch = pin-A0;
  return channels[ch].sampleCount;
}

uint16_t ADCManager::getValue(byte pin){  //return integer 0 to 4096 12 bit ADC
  /*byte ch = pin-A0;  
  channels[ch].convComplete = false;
  return channels[ch].value;*/
  if(pin==pinMotorRightSense) return rightCurVal;
  if(pin==pinMotorLeftSense) return leftCurVal;
}

float ADCManager::getVoltage(byte pin){
  uint16_t v = getValue(pin);
  //bb
   // return ((float)v) / ((float) ((1 << ADC_BITS)-1)) * ADC_REF;
  return ((float)v) / ((float) ((4096))) * ADC_REF;
}

void ADCManager::init(byte ch){

}

// start another conversion
void ADCManager::run(){
   extern int16_t sumx[2][255];
   extern int16_t posmin[2];
   extern int16_t posmax[2];
  if (millis() > nextPeri)
  {
    nextPeri = millis() + nextLoopTime;
    msend = millis() - msstart;
    msstart = millis();
    start_time = esp_timer_get_time();
    i2s_run(chanFlag & 1); // start DMA acquisition and copy perimeter data, also read 4 more ADC channels
    postProcess2(chanFlag & 1); // convert to 8bit raw data (needs little time ca. 0.1ms)
    robot.perimeter.matchedFilter(chanFlag & 1); // correlate with signal template on channel 0 (needs ca. 1ms)
    end_time = esp_timer_get_time() - start_time;

    if (doShow) // for full plotting
    {
      nextLoopTime = 3000;
      if (plot)
      {
        //if (millis() > nextPeriShow)
        {
          nextPeriShow = millis() + 2950;
          for (int i = 0; i < 10; i++)
          {
            if (chanFlag & 1) printf("0,10000.,0,0,0,-10000.\n"); // mark begin and scale plot range
            else printf("0,8000.,0,0,0,-8000.\n");
          }
          int j = 200;
          float k = 127.*2. / (ADCMax[chanFlag & 1] - ADCMin[chanFlag & 1]);
          if ((ADCMax[chanFlag & 1] == ADCMin[chanFlag & 1])) k = 1.;
          k = 25.;

          //printf("k=%f\n",k);

          for (int i = 0; i < SHOWSAMPLES; i++) // only show first SHOWSAMPLES
          {
            if ((j == 200) && (i == posmax[chanFlag & 1]))
            {
              /*if (chanFlag == 0)*/ j = 0;
            }
            printf("raw0:%f,sig:%f,ins0:%f,smag0:%f,mag0:%f,qual0:%f\n",
                   constrain(4000.*(k * 3. * (robot.perimeter.raw_buff[chanFlag & 1][i])  / 255.), - 4000, 4000),
                   4000.*(j < 24 * 4 ? 1.*sigcode_diff[(j >> 2)] : 0.),
                   robot.perimeter.isInside(chanFlag & 1) ? 6200. : -6200.,
                   //4000.*(1. * sumx[0][i] / 128.),
                   //(i == posmax[chanFlag&1]) ? 4000 * 2.3 : (i == posmin[chanFlag&1]) ? -4000 * 2.3 : 0./*(.3 * sumx[1][i] / 255.)*/,
                   20.*robot.perimeter.getSmoothMagnitude(chanFlag & 1),
                   10.*robot.perimeter.getMagnitude(chanFlag & 1),
                   1000.*robot.perimeter.getFilterQuality(chanFlag & 1));

            if (j < 24 * 4) j++;
          }
          for (int i = 0; i < 10; i++)
            printf("0,-0.5,0,0,0,0\n");

          printf("0,-5000,0,0,0,0\n");

          for (int i = 0; i < 10; i++)
            printf("0,0,0,0,0,0\n");
        }
      }
    }
    else // just plot result
    {
      nextLoopTime = 50;
      if (plot)
      {
        printf("sigcnt0:%f,tim:%f,ins0:%f,smag0:%f,mag0:%f,qual0:%f\n",
               100.*robot.perimeter.getSignalCounter(0),
               constrain((float)(1.*end_time), 0, 10000),
               robot.perimeter.isInside(0) ? 620. : -620.,
               constrain(10.*robot.perimeter.getSmoothMagnitude(0), -6100, 6100),
               constrain(10.*robot.perimeter.getMagnitude(0), -6100, 6100),
               1000.*robot.perimeter.getFilterQuality(0));
        //      printf("%f\n", 1.*end_time);
      }
    }
    ////chanFlag++; // toggle channel - except simple single perimeter coil
    //dacWrite(PWMPIN, chanFlag&0xff);       // PWM
    //dacWrite(PWMPIN2, chanFlag&0xf0);       // PWM

  }
}


void ADCManager::postProcess(byte ch){  
  //Console.print("post ch");
  //Console.print(ch);
  //Console.print("=");
  //Console.print(dmaData[0]);
  uint16_t vmax = 0;
  uint16_t vmin = 9999;   
  if (channels[ch].autoCalibrate) {  
    // determine zero point    
    for (int i=0; i < channels[ch].sampleCount; i++){
      uint16_t value = dmaData[i];
      vmax = max(vmax, value);
      vmin = min(vmin, value);
    }
    // determine gliding min,max,ofs
    channels[ch].maxValue = 0.9 * ((double)channels[ch].maxValue) + 0.1 * ((double)vmax);
    channels[ch].minValue = 0.9 * ((double)channels[ch].minValue) + 0.1 * ((double)vmin);
    channels[ch].zeroOfs = channels[ch].minValue + (channels[ch].maxValue - channels[ch].minValue)/2.0;
  }  
  // ------determine average value-------
  int32_t res = 0;
  int i;
  for (i=0; i < channels[ch].sampleCount; i++){
    uint16_t value = dmaData[i];    
    //Console.print(" v1=");
    //Console.print(value);    
    value -= channels[ch].zeroOfs;
    res += value;
  }
  //Console.print(" res=");
  //Console.print(res);    
  channels[ch].value = ((float)res) / ((float)channels[ch].sampleCount);      
  // --------transfer DMA samples----------
  for (int i=0; i < channels[ch].sampleCount; i++){
    uint16_t value = dmaData[i];
    value -= channels[ch].zeroOfs;
    channels[ch].samples[i] = min(SCHAR_MAX,  max(SCHAR_MIN, ((int8_t) (value >> (ADC_BITS-8)))+0 )); // convert to 8 bits
  }
  //Console.print(" val");
  //Console.println(channels[ch].value);
}
