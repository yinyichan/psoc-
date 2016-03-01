#include <FS.h>
#include <project.h>
#include <string.h>
#include <Global.h>
#include <stdlib.h>
#include <stdio.h>

uint8 InterruptCnt;
uint8 Flag_If_Int=0;

#include <math.h>

CY_ISR(InterruptHandler)
{
	/* Read Status register in order to clear the sticky Terminal Count (TC) bit 
	 * in the status register. Note that the function is not called, but rather 
	 * the status is read directly.
	 */
   	Timer_1_STATUS;
    
	/* Increment the Counter to indicate the keep track of the number of 
     * interrupts received */
    InterruptCnt++;	
	
	//flag if interrupt occured
	Flag_If_Int=1;
}

void MeasureTempHumidity(float *temp_inside, float *hum_inside);
void formatTempHumData();
void printToFile();
void sendToUART();

char sdFile[9] = "Temp.txt";
char sdFileHum[9] = "Hum.txt";
char sdFileLight[9] = "Light.txt";
char tempTotString[23], humTotString[23], lightTotString[24];
float tempTemp, tempHum, tempLight;
uint8 prevYear, prevMonth, prevDay, prevHour, prevMinute, prevSecond; 
uint8 tempData[2];
FS_FILE * pFile, * pFileHum, * pFileLight;
uint8 tempSend1, tempSend2, humSend1, humSend2, lightSend1, lightSend2;

int32 filt = 0;
int32 LowPassFilter(int32 input);

int main()
{
    float temp, hum, light;
    
    ADC_SAR_Seq_Start(); //start ADC
    ADC_SAR_Seq_StartConvert(); //Start conversions
    ADC_SAR_Seq_IsEndConversion(ADC_SAR_Seq_WAIT_FOR_RESULT); //necessary ADC function
    UART_1_Start(); //start UART
    CyGlobalIntEnable; //enable Global interrupts
    
    RTC_Start(); //start real time clock
 	RTC_WriteIntervalMask(RTC_INTERVAL_SEC_MASK | RTC_INTERVAL_MIN_MASK);
    RTC_WriteYear(0); //write year
    RTC_WriteMonth(0); //write month
    RTC_WriteDayOfMonth(0); //write day
    RTC_WriteHour(0); //write hour
    RTC_WriteMinute(0); //write minute
    RTC_WriteSecond(0); //write second
    
    Inside_I2C_Start(); //start I2C block

    TimerISR_StartEx(InterruptHandler); //start interrupt function
	Timer_Reset_Write(1); //enable reset
    Timer_1_Start(); //start timer
	
    FS_Init(); //enable SD card file system
    
    pFile = FS_FOpen(sdFile, "w"); //open temp file
    pFileHum = FS_FOpen(sdFileHum, "w"); //open humidity file
    pFileLight = FS_FOpen(sdFileLight, "w"); //open light file
    
    MeasureTempHumidity(&temp, &hum); //measure initial temperature and humidity
    light = ADC_SAR_Seq_CountsTo_Volts(ADC_SAR_Seq_GetResult16()) * 10 / 0.023; //measure initial light data
    tempTemp = temp; //save temp for formatting
    tempHum = hum; //save humidity for formatting
    tempLight = light; //save light data for formatting
    formatTempHumData(); //formate all data
    printToFile(); //print to SD card files
    sendToUART(); //send to PSoC 4 to make available over bluetooth

    for(;;)
    {
        light = ADC_SAR_Seq_CountsTo_Volts(ADC_SAR_Seq_GetResult16()) * 10 / 0.023; //measure light data
        filt = LowPassFilter(light); //apply a digital filter to light ADC measurement
        if(Flag_If_Int == 1 && InterruptCnt != 10)
		{
			//flag if interrupt occured
			Flag_If_Int=0;
			
			//start agian the one shot timer.
			Timer_Reset_Write(1);
		}		
        if(Flag_If_Int == 1 && InterruptCnt == 10) {
            Flag_If_Int = 0; //flag if interrupt occured
            InterruptCnt = 0;
            Timer_Reset_Write(1); //start agian the one shot timer.
            
            MeasureTempHumidity(&temp, &hum); //measure temp and humidity
            light = ADC_SAR_Seq_CountsTo_Volts(ADC_SAR_Seq_GetResult16()) * 10 / 0.023; //measure light data
            filt = LowPassFilter(light); //filter the data
            tempTemp = temp; //store for formatting
            tempHum = hum; //store humidity for formatting
            tempLight = filt; //store light for formatting
            formatTempHumData(); //formate data
            printToFile(); //print to SD card
            sendToUART(); //send to PSoC 4 to make temperature available over bluetooth
        }
    }
}

void formatTempHumData() 
{
    prevYear = RTC_ReadYear(); //read year from RTC
    prevMonth = RTC_ReadMonth(); //read month from RTC
    prevDay = RTC_ReadDayOfMonth(); //read day from RTC
    prevHour = RTC_ReadHour(); //read hour from RTC
    prevMinute = RTC_ReadMinute(); //read minute from RTC
    prevSecond = RTC_ReadSecond(); //read second from RTC
    
    int intTemp = (int)tempTemp; //find integer from temperature
    int decTemp = ((int)(tempTemp*100)%100); //formate decimal of temperature
    sprintf(tempTotString, "%02d/%02d/%02d %02d:%02d:%02d %02d.%02d", prevMonth, prevDay, prevYear, prevHour, prevMinute, prevSecond, intTemp, decTemp);
    
    int intHum = (int)tempHum; //formate humidity data
    int decHum = ((int)(tempHum*100)%100); //format humidity data
    sprintf(humTotString, "%02d/%02d/%02d %02d:%02d:%02d %02d.%02d", prevMonth, prevDay, prevYear, prevHour, prevMinute, prevSecond, intHum, decHum);
    
    int intLight = (int)tempLight; //format light data
    int decLight = ((int)(tempLight*100)%100); //format light data
    sprintf(lightTotString, "%02d/%02d/%02d %02d:%02d:%02d %02d.%03d", prevMonth, prevDay, prevYear, prevHour, prevMinute, prevSecond, intLight, decLight);
}

void MeasureTempHumidity(float *temp_inside, float *hum_inside)
{
     /*Values for read humidity*/
    uint8 T_LSB_Inside = 0, H_LSB_Inside = 0, T_MSB_Inside = 0, H_MSB_Inside = 0;    
	
	 /*I2C Commands for capturing humidity*/
    uint8 Slave_Addr = 0x27;
	
	 /*Buffer for reading humidity values*/
    uint8 Write_Buf_Inside[1] = {0};;    
    uint8 Read_Buf_Inside[4] = {0};
    uint8 status;
    
	/*Read and write values*/
    Inside_I2C_MasterWriteBuf(Slave_Addr, Write_Buf_Inside, 1, Inside_I2C_MODE_COMPLETE_XFER);
    while((Inside_I2C_MasterStatus() & 0x02) != 2);    
    status = 1;
    while (status != 0) {
        Inside_I2C_MasterReadBuf(Slave_Addr, Read_Buf_Inside, 4, Inside_I2C_MODE_COMPLETE_XFER);
        while((Inside_I2C_MasterStatus() & 0x01) != 1);
        status = (Read_Buf_Inside[0] & 0xC0); 
    }
	
	H_MSB_Inside = Read_Buf_Inside[0] & 0x3F;
    H_LSB_Inside = Read_Buf_Inside[1];
    *hum_inside = ((H_MSB_Inside*256)+H_LSB_Inside) * 100 / (pow(2,14) - 2);
    T_MSB_Inside = Read_Buf_Inside[2];
    T_LSB_Inside = Read_Buf_Inside[3] >> 2;
    *temp_inside = (((T_MSB_Inside*64)+T_LSB_Inside) * 165 / (pow(2,14) - 2)) - 40;
}

void printToFile()
{
    if(pFile)
    {
        if(0 != FS_Write(pFile, tempTotString, 23)) 
        {
            FS_Write(pFile, "\r\n", 2);
        }
    }
    if(pFileHum)
    {
        if(0 != FS_Write(pFileHum, humTotString, 23)) 
        {
            FS_Write(pFileHum, "\r\n", 2);
        }
    }
    if(pFileLight)
    {
        if(0 != FS_Write(pFileLight, lightTotString, 24)) 
        {
             FS_Write(pFileLight, "\r\n", 2);
        }
    }
}

void sendToUART()
{
    tempSend1 = (int)tempTemp; //format data to be sent over UART
    tempSend2 = ((int)(tempTemp*100)%100);
    humSend1 = (int)tempHum;
    humSend2 = ((int)(tempHum*100)%100);
    lightSend1 = (int)tempLight;
    lightSend2 = ((int)(tempLight*100)%100);
    RTS_Write(1);
    while(CTS_Read() != 1);
    UART_1_PutChar(tempSend1);
    UART_1_PutChar(tempSend2);
    UART_1_PutChar(humSend1);
    UART_1_PutChar(humSend2);
    UART_1_PutChar(lightSend1);
    UART_1_PutChar(lightSend2);
    RTS_Write(0);
}

int32 LowPassFilter(int32 input)
{
    int32 k;
    input <<= 8;
    filt = filt + (((input-filt) >> 8) * 17);
    k = (filt>>8) + ((filt & 0x00000080) >> 7);
    return k;
}


/* [] END OF FILE */
