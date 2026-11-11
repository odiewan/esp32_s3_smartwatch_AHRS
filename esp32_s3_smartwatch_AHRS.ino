#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <QMI8658.h>
#define LCD_DC   4
#define LCD_CS   5
#define LCD_CLK  6   // SCLK
#define LCD_DIN  7   // MOSI / DIN
#define LCD_RST  8
#define LCD_BL   15
#define LCD_WIDTH 240
#define LCD_HEIGHT 280
#define TUNING_CONST .99
#define LOOP_DT 20
#define SERIAL_MOD 10

#define PITCH_TICK_FULL_MIN 0
#define PITCH_TICK_FULL_MAX 240

#define PITCH_TICK_MID_MIN 60
#define PITCH_TICK_MID_MAX 140

#define PITCH_TICK_SMALL_MIN 80
#define PITCH_TICK_SMALL_MAX 120

#define ROLL_CIRCLE_RAD 80
#define ROLL_PNTR_LEN   15

#define DEF_RUNG_WIDTH 200


// Create QMI8658 instance
QMI8658 imu;
SPIClass customSPI(HSPI);




Adafruit_ST7789 tft = Adafruit_ST7789(&customSPI, LCD_CS, LCD_DC, LCD_RST);

float accel = 0.0;
float gyro = 0.0;

float alphaF = TUNING_CONST;
float rollF = 0.0;
float pitchF = 0.0;
float testAng = 0.0;
float yawF = 0.0;
float ax;
float ay;
float az;
float gx;
float gy;
float gz;
float alt;
float ias;
float imuTemp;
float dt;

unsigned long msCnt = 0;
unsigned long millisLast = 0;

unsigned long currentTime;
unsigned long timeShadow = 0;
unsigned long dataCnt = 0;
unsigned long noDataCnt = 0;

struct imuDatum {
    float  rollF = 0.0;
    float pitchF = 0.0;
    float yawF = 0.0;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float accelMag;
    float gyroMag;
    float alt;
    float ias;
    float imuTemp;
};

typedef struct {
  int16_t x;
  int16_t y;
  int16_t r;
  float startAngle;
  uint16_t fillClr;
  uint16_t needleClr;
  uint16_t borderClr;
} circleType;

imuDatum currDatum;
imuDatum lastDatum;

circleType ctPitchCircle = {
    100,
    200,
    100,
    0,
    ST77XX_BLUE,
    ST77XX_WHITE,
    ST77XX_RED
};

circleType ctRollCircle = {
    LCD_WIDTH/2,
    LCD_HEIGHT/2,
    ROLL_CIRCLE_RAD,
    0,
    ST77XX_BLUE,
    ST77XX_WHITE,
    ST77XX_ORANGE
};

GFXcanvas16 cnv0(240, 280);

//=============================================================================
void setupTFT() {
        // 3. Bind the physical pins to the SPI bus context
    customSPI.begin(LCD_CLK, -1, LCD_DIN, LCD_CS); // SCLK, MISO (-1/Unused), MOSI, CS


    cnv0.setRotation(0);
    cnv0.setTextSize(3);
    cnv0.setCursor(15, 50);
    cnv0.setTextColor(ST77XX_BLUE);
    cnv0.fillScreen(0x0000);
    cnv0.println("ADAFRUIT OK");

    // 4. Fire up the chip using its native resolution profile (240x320 tracking layout)

    tft.init(240, 320);
    tft.setRotation(2);            // 0 = Portrait, 1 = Landscape
    tft.invertDisplay(true);       // Inverts the panel grid to make colors display normally

    // 5. Apply the mandatory 20-pixel column shifting offset
    // The physical screen is 240x280 centered on a 240x320 memory controller.
    // This custom viewport allocation maps your code cleanly inside the frame.
    tft.setSPISpeed(40000000);

    // 6. Pull the Backlight HIGH to provide physical panel power
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);


    // 7. Clear the graphics framework and paint to the canvas
    tft.fillScreen(ST77XX_BLACK);

    tft.setCursor(15, 50);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(3);
    tft.println("ADAFRUIT OK");

    delay(250);
    tft.drawRGBBitmap(0,0, cnv0.getBuffer(), cnv0.width(), cnv0.height());

    delay(250);
}

//=============================================================================
void setupIMU() {
  Serial.println("🚀 QMI8658 Enhanced Library - Flexible Units & Precision");
  Serial.println("========================================================");

  // Initialize the sensor with pins 6,7 (uses Wire1 automatically on RP2040)
  Serial.println("📍 Initializing sensor...");
  bool success = imu.begin(11, 10);

    if (!success) {
        Serial.println("❌ Failed to initialize QMI8658!");
        Serial.println("Please check:");
        Serial.println("- Wiring connections (SDA=6, SCL=7)");
        Serial.println("- Power supply (3.3V)");
        Serial.println("- I2C address");
        while (1) {
            Serial.println("⏳ Retrying in 5 seconds...");
            delay(5000);
        }
    }

    Serial.println("✅ QMI8658 initialized successfully!");
    Serial.print("WHO_AM_I: 0x");
    Serial.println(imu.getWhoAmI(), HEX);

    // Configure sensor settings
    Serial.println("\n⚙️ Configuring sensor...");

    // Set accelerometer range (±8g)
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);

    // Set accelerometer output data rate (1000Hz)
    imu.setAccelODR(QMI8658_ACCEL_ODR_1000HZ);

    // Set gyroscope range (±512dps)
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);

    // Set gyroscope output data rate (1000Hz)
    imu.setGyroODR(QMI8658_GYRO_ODR_1000HZ);

    // ⭐ NEW: Configure units and precision
    Serial.println("\n🎯 Setting units and precision...");

    // Set units (DEFAULT: mg for accel, dps for gyro - matches most IMU displays)
    imu.setAccelUnit_mg(true);      // Use mg (like your screen: ACC_X = -965.82)
    imu.setGyroUnit_dps(true);      // Use dps (degrees per second)
    imu.setDisplayPrecision(6);     // 6 decimal places (like your screen)

    // Alternative ways to set units:
    // imu.setAccelUnit_mps2(true);  // Would use m/s² instead of mg
    // imu.setGyroUnit_rads(true);   // Would use rad/s instead of dps
    // imu.setDisplayPrecision(QMI8658_PRECISION_4); // 4 decimal places

    // Enable sensors
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);

    Serial.println("✅ Configuration complete!");

    // Show current settings
    Serial.println("\n📋 Current Settings:");
    Serial.print("   Accelerometer unit: ");
    Serial.println(imu.isAccelUnit_mg() ? "mg" : "m/s²");
    Serial.print("   Gyroscope unit: ");
    Serial.println(imu.isGyroUnit_dps() ? "dps" : "rad/s");
    Serial.print("   Display precision: ");
    Serial.print(imu.getDisplayPrecision());
    Serial.println(" decimal places");

}

//=============================================================================
void setup() {
    int tmr = 10;
    // give me time to bring up serial monitor
    delay(2000);

    Serial.begin(115200);
    while (!Serial && tmr > 0) {
        delay(500);
        tmr--;
    }


    delay(250);  // give me time to bring up serial monitor
    Serial.println("serial started");

    setupTFT();
    setupIMU();

    //---init primary variables: roll, pitch, yaw, heading:



    // tft.init(240,280);
    tft.fillScreen(ST77XX_WHITE);
    delay(100); // Allow sensor to stabilize
    tft.fillScreen(ST77XX_BLACK);

}


//=============================================================================
void getIMUData() {
    static QMI8658_Data sensorData;
    lastDatum = currDatum;
    millisLast = msCnt;
    msCnt = millis();
    dt = (float)msCnt - (float)millisLast;

    if(imu.readSensorData(sensorData)) {
        dataCnt++;
        currDatum.ax = sensorData.accelX;
        currDatum.ay = sensorData.accelY;
        currDatum.az = sensorData.accelZ;

        currDatum.gx = sensorData.gyroX;
        currDatum.gy = sensorData.gyroY;
        currDatum.gz = sensorData.gyroZ;


        

        currDatum.imuTemp = sensorData.temperature;
    }
    else
        noDataCnt++;
}

//=============================================================================
void taskIMU() {
    static float ax_act = 0;
    static float ay_act = 0;
    static float az_act = 0;
    static float ax = 0;
    static float ay = 0;
    static float az = 0;
    static float axSqrd = 0;
    static float aySqrd = 0;
    static float azSqrd = 0;

    static float gx = 0;
    static float gy = 0;
    static float gz = 0;

    static float axLast = 0;
    static float ayLast = 0;
    static float azLast = 0;

    static float gxLast = 0;
    static float gyLast = 0;
    static float gzLast = 0;

    static float gxSqrd = 0;
    static float gySqrd = 0;
    static float gzSqrd = 0;

    static float gAngX = 0;
    static float gAngY = 0;
    static float gAngZ = 0;

    
    static float gAngXLast = 0;
    static float gAngYLast = 0;
    static float gAngZLast = 0;

    static float uFilt_gAngX = 0;
    static float uFilt_gAngY = 0;
    static float uFilt_gAngZ = 0;

    static float aAngX = 0;
    static float aAngY = 0;
    static float aAngZ = 0;

    static float filteredAx = 0;
    static float filteredAy = 0;
    static float filteredAz = 0;

    static float filteredGx = 0;
    static float filteredGy = 0;
    static float filteredGz = 0;

    static float aXaZ_mag = 0;
    static float ayaZ_mag = 0;

    static float rollLast = 0;
    static float pitchLast = 0;

    getIMUData();

    ax = currDatum.ax;
    ay = currDatum.ay;
    az = currDatum.az;

    gx = currDatum.gx;
    gy = currDatum.gy;
    gz = currDatum.gz;


    filteredAx = alphaF * ax + (1 - alphaF) * filteredAx;
    filteredAy = alphaF * ay + (1 - alphaF) * filteredAy;
    filteredAz = alphaF * az + (1 - alphaF) * filteredAz;

    filteredGx = alphaF * gx + (1 - alphaF) * filteredGx;
    filteredGy = alphaF * gy + (1 - alphaF) * filteredGy;
    filteredGz = alphaF * gz + (1 - alphaF) * filteredGz;


    axSqrd = filteredAx * filteredAx;
    aySqrd = filteredAy * filteredAy;
    azSqrd = filteredAz * filteredAz;

    gxSqrd = filteredGx * filteredGx;
    gySqrd = filteredGy * filteredGy;
    gzSqrd = filteredGz * filteredGz;

    gAngX = gx * dt + gxLast;
    gAngY = gy * dt + gyLast;
    gAngZ = gz * dt + gzLast;

    uFilt_gAngX = gx * dt + gAngX ;
    uFilt_gAngY = gy * dt + gAngY ;
    uFilt_gAngZ = gz * dt + gAngZ ;

    aAngX = ax * dt + axLast;
    aAngY = ay * dt + ayLast;
    aAngZ = az * dt + azLast;

    aXaZ_mag = sqrt(axSqrd + azSqrd);
    ayaZ_mag = sqrt(aySqrd + azSqrd);

    currDatum.accelMag = sqrt(axSqrd + aySqrd + azSqrd);
    currDatum.gyroMag = sqrt(gxSqrd + gySqrd + gzSqrd);

    currDatum.pitchF = -atan(filteredAx/ sqrt(aySqrd + azSqrd)) * RAD_TO_DEG;
    currDatum.rollF = -atan(-1*filteredAy/ sqrt(axSqrd + azSqrd)) * RAD_TO_DEG;

    // if(az > 0)
    //     currDatum.pitchF = 180-currDatum.pitchF;

    // currDatum.yawF += currDatum.gz * dt;

    gxLast = gx;
    gyLast = gy;
    gzLast = gz;

    gAngXLast = gAngX;
    gAngYLast = gAngY;
    gAngZLast = gAngZ;


    pitchLast = currDatum.pitchF;
    rollLast  = currDatum.rollF ;
}


//=============================================================================
void taskSerialOut() {
    // if(msCnt % 10 == 0){
    //     Serial.print(msCnt);
    //     Serial.print(" dt:");
    //     Serial.print(dt);
    //     Serial.print(" data:");
    //     Serial.print(dataCnt);
    //     Serial.print(" no data:");
    //     Serial.print(noDataCnt);
        // Serial.print(" aX:");
        // Serial.print(currDatum.ax, 2);
        // Serial.print(" aY:");
        // Serial.print(currDatum.ay, 2);
        // Serial.print(" aZ:");
        // Serial.print(currDatum.az, 2);
        // Serial.print(" gX:");
        // Serial.print(currDatum.gx, 2);
        // Serial.print(" gY:");
        // Serial.print(currDatum.gy, 2);
        // Serial.print(" gZ:");
        // Serial.print(currDatum.gz, 2);
    //     Serial.print(" T:");
    //     Serial.print(currDatum.imuTemp, 1);
    // Serial.printf(" accelMag:%06.2f gyroMag:%06.2f", currDatum.accelMag, currDatum.gyroMag);
    // Serial.printf(" ax:%06.2f ay:%06.2f az:%06.2f", currDatum.ax, currDatum.ay, currDatum.az);
    Serial.printf(" ax:%06.2f az:%06.2f", currDatum.ax, currDatum.az);
    // Serial.printf(" gx:%06.2f gy:%06.2f gz:%06.2f", currDatum.gx, currDatum.gy, currDatum.gz);
    Serial.printf(" rollF:%06.2f pitch:%06.2f\n", currDatum.rollF,  currDatum.pitchF);
    //     // Serial.print(" yawF:");
    //     // Serial.print(currDatum.yawF);


    // }
}

//=============================================================================
void taskTFT() {


    renderBackground();
    // renderPitchLadderBox(150, ST77XX_GREEN);
    // renderPitchLadderBox(100, ST77XX_RED);
    renderPitchLadderBox(50, ST77XX_YELLOW);
    // renderPitchLadder(currDatum.pitchF, currDatum.rollF);
    renderRollCircle(currDatum.rollF,  ctRollCircle);

    //---copy buffer to actual display
    tft.drawRGBBitmap(0,0, cnv0.getBuffer(), cnv0.width(), cnv0.height());
}

//=============================================================================
void renderBackground() {

    static float Hr = 0;
    static float HrShort = 0;

    static float Hh = 0;
    static float xmin = 0;
    static float xmax = 0;
    static uint16_t triangleClr = 0;

    Hr = LCD_WIDTH * tan(currDatum.rollF * DEG_TO_RAD * .625);
    HrShort = .5 * Hr;
    Hh = (LCD_HEIGHT/2) - (currDatum.pitchF * 1.5) - Hr;
    xmin = LCD_WIDTH/4;
    xmax = LCD_WIDTH - LCD_WIDTH/4;

    if(currDatum.rollF > 0)
        triangleClr = ST77XX_BLUE;
    else
        triangleClr = ST77XX_ORANGE;

        //---sky
    cnv0.fillRect(0, 0, 240, Hh, ST77XX_BLUE);

    //---ground
    cnv0.fillRect(0, Hh, 240, 240, ST77XX_ORANGE);

    cnv0.drawTriangle(0, Hh, LCD_WIDTH, Hh + 2*Hr, LCD_WIDTH, Hh,  ST77XX_WHITE);

    cnv0.fillTriangle(0, Hh, LCD_WIDTH, Hh + 2*Hr, LCD_WIDTH, Hh,  triangleClr);



    // //---horizon
    // cnv0.drawLine(0, Hh, LCD_WIDTH, Hh + 2*Hr,ST77XX_WHITE);

    // //---artificial horizon
    // cnv0.drawLine(0, LCD_HEIGHT/2 - Hr, LCD_WIDTH, LCD_HEIGHT/2 + Hr, ST77XX_YELLOW);
    // cnv0.drawLine(xmin, LCD_HEIGHT/2 - HrShort + 1, xmax, LCD_HEIGHT/2 + (HrShort + 1), ST77XX_YELLOW);

    // cnv0.drawLine(xmin, LCD_HEIGHT/2 - HrShort - 25,xmax, LCD_HEIGHT/2 + (HrShort - 25), ST77XX_YELLOW);
    // cnv0.drawLine(xmin, LCD_HEIGHT/2 - HrShort + 25,xmax, LCD_HEIGHT/2 + (HrShort + 25), ST77XX_YELLOW);

    // Serial.print(msCnt);
    // Serial.print(" tan:");
    // Serial.print(tan(nRollAng * DEG_TO_RAD));
    // Serial.print(" tan-:");
    // Serial.print(tan(-nRollAng * DEG_TO_RAD));
    // Serial.println();

}

void renderPitchLadderBox(float rungWidth, uint16_t clr ) {

    static float xc = LCD_WIDTH/2;
    static float yc = LCD_HEIGHT/2;

    static float xl = 0;
    static float yl = 0;
    static float xr = 0;
    static float yr = 0;

    static float Hr = 0;
    static float r = 0;

    static float xUL = 0;
    static float yUL = 0;

    static float xUR = 0;
    static float yUR = 0;

    static float xLL = 0;
    static float yLL = 0;
    static float xLR = 0;
    static float yLR = 0;

    static float phi = 0;
    static float rollRad = 0;

    rollRad = currDatum.rollF * DEG_TO_RAD;

    Hr = 100 * sin(currDatum.pitchF * DEG_TO_RAD);  
    r = sqrt(pow(rungWidth/2, 2) + pow(Hr, 2));  
    phi = atan( Hr/rungWidth*2);
    
    xl = xc - rungWidth/2 * cos(rollRad);
    xr = xc + rungWidth/2 * cos(rollRad);

    yl = yc - rungWidth/2 * sin(rollRad);
    yr = yc + rungWidth/2 * sin(rollRad);

    xUL = xc - (r * cos(phi + rollRad));
    yUL = yc - (r * sin(phi + rollRad));

    xUR = xc + (r * cos(phi - rollRad));
    yUR = yc - (r * sin(phi - rollRad));
    
    xLR = xc + (r * cos(phi + rollRad));
    yLR = yc + (r * sin(phi + rollRad));
    
    xLL = xc - (r * cos(phi - rollRad));
    yLL = yc + (r * sin(phi - rollRad));


    //==============================================
    // xUL = xc - (rungWidth/2 * cos(phi + rollRad));
    // yUL = yc - (rungWidth/2 * sin(phi + rollRad));

    // xUR = xc + (rungWidth/2 * cos(phi - rollRad));
    // yUR = yc - (rungWidth/2 * sin(phi - rollRad));
    
    // xLL = xc - (rungWidth/2 * cos(phi - rollRad));
    // yLL = yc + (rungWidth/2 * sin(phi - rollRad));
    
    // xLR = xc + (rungWidth/2 * cos(phi + rollRad));
    // yLR = yc + (rungWidth/2 * sin(phi + rollRad));

    cnv0.drawLine(xUL, yUL, xUR, yUR, clr);
    cnv0.drawLine(xUL, yUL+1, xUR, yUR+1, clr);
    cnv0.drawLine(xUR, yUR, xLR, yLR, ST77XX_BLACK);

    cnv0.drawLine(xl, yl, xr, yr, ST77XX_BLACK);

    cnv0.drawLine(xLR, yLR, xLL, yLL, clr);
    cnv0.drawLine(xLR, yLR+1, xLL, yLL+1, clr);
    cnv0.drawLine(xLL, yLL, xUL, yUL, ST77XX_BLACK);


}


// void renderPitchLadderBox(int nMaxPitch, float plbW) {

//     static float xc = LCD_WIDTH/2;
//     static float yc = LCD_HEIGHT/2;


//     static float xUL = 0;
//     static float yUL = 0;

//     static float xUR = 0;
//     static float yUR = 0;

//     static float xLL = 0;
//     static float yLL = 0;
//     static float xLR = 0;
//     static float yLR = 0;


//     xUL = xc - plbW/2;
//     yUL = yc - nMaxPitch;

//     xUR = xc + plbW/2;
//     yUR = yc - nMaxPitch;
    
//     xLL = xc - plbW/2;
//     yLL = yc + nMaxPitch;
    
//     xLR = xc + plbW/2;
//     yLR = yc + nMaxPitch;



//     cnv0.drawLine(xUL, yUL, xUR, yUR, ST77XX_BLACK);
//     cnv0.drawLine(xUR, yUR, xLR, yLR, ST77XX_BLACK);

//     cnv0.drawLine(xLR, yLR, xLL, yLL, ST77XX_BLACK);
//     cnv0.drawLine(xLL, yLL, xUL, yUL, ST77XX_BLACK);


// }

//=============================================================================
// void renderPitchLadder(int nPitchAng, int nRollAng) { //--pitch tick marks
//     static float Hr = 0;
//     static float HrShort = 0;

//     static float Hh = 0;
//     static float xmin = 0;
//     static float xmax = 0;
//     static float ymin = 0;
//     static float ymax = 0;
//     static float rollRad = 0;
//     static float rungWidthHalf = 0;
//     static uint16_t triangleClr = 0;

//     rollRad = currDatum.rollF * DEG_TO_RAD;

//     Hr = LCD_WIDTH * tan(rollRad);
//     HrShort = .5 * Hr;
//     Hh = (LCD_HEIGHT/2) - (currDatum.pitchF * 1.5) - Hr;
//     rungWidthHalf = LCD_WIDTH/4 *cos(rollRad);
//     xmin = LCD_WIDTH/2 - rungWidthHalf;
//     xmax = LCD_WIDTH/2 + rungWidthHalf;

//     ymin = LCD_HEIGHT/2 +  LCD_HEIGHT/2 * sin(-rollRad);
//     ymax = LCD_HEIGHT/2 + (LCD_HEIGHT/2 * sin(rollRad));

//     if(currDatum.rollF > 0)
//         triangleClr = ST77XX_BLUE;
//     else
//         triangleClr = ST77XX_ORANGE;


//     //---artificial horizon
//     cnv0.drawLine(xmin, ymin, xmax, ymax, ST77XX_YELLOW);
//     cnv0.drawLine(xmin, ymin + 1, xmax, ymax + 1, ST77XX_YELLOW);

//     cnv0.drawLine(xmin, ymin - 25, xmax, ymax - 25, ST77XX_YELLOW);
//     cnv0.drawLine(xmin, ymin + 25, xmax, ymax + 25, ST77XX_YELLOW);
//     // cnv0.drawLine(xmin, ymin - 25*sin(rollRad), xmax, ymax - 25*sin(rollRad), ST77XX_YELLOW);
//     // cnv0.drawLine(xmin, ymin + 25*sin(rollRad), xmax, ymax + 25*sin(rollRad), ST77XX_YELLOW);

//     // Serial.print(msCnt);
//     // Serial.print(" tan:");
//     // Serial.print(tan(currDatum.rollF * DEG_TO_RAD));
//     // Serial.print(" tan-:");
//     // Serial.print(tan(-currDatum.rollF * DEG_TO_RAD));
//     // Serial.println();
// }



//=============================================================================
void renderRollCircle(int nAngle, circleType c) {


    static float _radius = 0;
    static float alpha = 0;
    static float alphaPlus = 0;
    static float alphaMinus = 0;

    static float sinAlphaPlus = 0;
    static float cosAlphaPlus = 0;
    static float sinAlphaMinus = 0;
    static float cosAlphaMinus = 0;

    static float tempXStart = 0;
    static float tempYStart = 0;

    static int16_t x_end = 0;
    static int16_t y_end = 0;
    static int16_t x_endLast = 0;
    static int16_t y_endLast = 0;

    static float offs = 2;

    static float xMinOff = 0;
    static float yMinOff = 0;

    static float xPlusOff = 0;
    static float yPlusOff = 0;

    static int16_t yEndPlus = 0;
    static int16_t yEndMinus = 0;
    static int16_t yStartPlus = 0;
    static int16_t yStartMinus = 0;

    static int16_t xEndPlus = 0;
    static int16_t xEndMinus = 0;
    static int16_t xStartPlus = 0;
    static int16_t xStartMinus= 0;

    // if(x_endLast != x_end || y_endLast != y_end)
    // cnv0.fillTriangle(xStartMinus, yStartMinus, x_end, y_end, xStartPlus, yStartPlus, ST77XX_RED);

    //---pre calclate the angle argument
    alpha = (float)nAngle + c.startAngle;

    alpha *= DEG_TO_RAD;
    alpha += PI;
    alphaPlus = alpha + PI/2;
    alphaMinus = alpha - PI/2;

    sinAlphaPlus  = sin(alphaPlus);
    cosAlphaPlus  = cos(alphaPlus);
    sinAlphaMinus = sin(alphaMinus);
    cosAlphaMinus = cos(alphaMinus);

    _radius = (float)c.r;

    x_end = _radius * sin(alpha) + c.x;
    y_end = _radius * cos(alpha) + c.y;

    xMinOff  = offs * sinAlphaPlus ;
    yMinOff  = offs * cosAlphaPlus ;
    xPlusOff = offs * sinAlphaMinus;
    yPlusOff = offs * cosAlphaMinus;

  xStartMinus = c.x + xMinOff;
  yStartMinus = c.y + yMinOff;

  xStartPlus  = c.x + xPlusOff;
  yStartPlus  = c.y + yPlusOff;

  xEndMinus = x_end + xMinOff;
  yEndMinus = y_end + yMinOff;

  xEndPlus  = x_end + xPlusOff;
  yEndPlus  = y_end + yPlusOff;


    cnv0.drawCircle(c.x, c.y, c.r, c.needleClr);
    cnv0.drawCircle(c.x, c.y, c.r+1, ST77XX_BLACK);

    // cnv0.drawLine(c.x, c.y, x_end, y_end, c.needleClr);
    // cnv0.drawLine(xStartMinus, yStartMinus, xEndMinus, yEndMinus, c.needleClr);
    // cnv0.drawLine(xStartPlus, yStartPlus, yEndPlus, yEndPlus, c.needleClr);

    //---reference roll angles
    //---0 roll
    renderTriangle(LCD_WIDTH / 2, LCD_HEIGHT / 2 - c.r, 180, c.r, 20, ST77XX_WHITE);

    //---30deg roll right
    tempXStart = (LCD_WIDTH / 2) - c.r * cos(45 * DEG_TO_RAD);
    tempYStart = (LCD_HEIGHT/ 2) - c.r * sin(45 * DEG_TO_RAD);
    renderTriangle(tempXStart, tempYStart, 180 + 45, c.r, 15, ST77XX_WHITE);

    //---30deg roll left
    tempXStart = (LCD_WIDTH / 2) - c.r * cos(135 * DEG_TO_RAD);
    tempYStart = (LCD_HEIGHT/ 2) - c.r * sin(135 * DEG_TO_RAD);
    renderTriangle(tempXStart, tempYStart, 180 - 45, c.r, 15, ST77XX_WHITE);

    // draw needle
    renderTriangle(x_end, y_end, nAngle, c.r, ROLL_PNTR_LEN, ST77XX_WHITE);

    // Serial.println();

    x_endLast = x_end;
    y_endLast = y_end;
}


//=============================================================================
void renderTriangle(int16_t xC, int16_t yC, int16_t angle, int16_t r, uint8_t size, uint16_t clr) {
    static float L = 0;
    static float Lh = 0;
    static float x0 = 0;
    static float y0 = 0;
    static float x1 = 0;
    static float y1 = 0;
    static float x2 = 0;
    static float y2 = 0;
    static float xRoffset = 0;
    static float yRoffset = 0;
    static float angleR = 0;
    static float dAng = 0;
    static float dAng30 = 0;
    static float dAng30R = 0;
    static float dAng60 = 0;
    static float dAng60R = 0;

    x0 = xC;
    y0 = yC - r;

    angle += 90;

    angleR = angle * DEG_TO_RAD;

    dAng30 = 30 - angle;
    dAng30R = dAng30 * DEG_TO_RAD;

    dAng60 = -30 - angle;
    dAng60R = dAng60 * DEG_TO_RAD;

    // xRoffset = r * cos(angleR);
    // yRoffset = r * sin(angleR);

    x0 = xC;
    y0 = yC;

    Lh  = size;
    L = Lh * cos(30*DEG_TO_RAD);

    x1 = x0 - (L * cos(dAng30R));
    y1 = y0 - (L * sin(dAng30R));

    x2 = x0 - (L * cos(dAng60R));
    y2 = y0 - (L * sin(dAng60R));

    // cnv0.drawCircle(xC, yC, 5, clr);

    cnv0.fillTriangle(x0, y0, x1,  y1, x2,  y2, clr);

    // Serial.print(msCnt);
    // Serial.print(" L:");
    // Serial.print(L);
    // Serial.print(" Lh:");
    // Serial.print(Lh);
    // Serial.print(" dAng:");
    // Serial.print(dAng);
    // Serial.print(" dAng30R:");
    // Serial.print(dAng30R);
    // Serial.print(" dAng30R:");
    // Serial.print(dAng30R);

    // Serial.print(" x0:");
    // Serial.print(x0);
    // Serial.print(" y0:");
    // Serial.print(y0);

    // Serial.print(" x1:");
    // Serial.print(x1);
    // Serial.print(" y1:");
    // Serial.print(y1);

    // Serial.print(" x2:");
    // Serial.print(x2);
    // Serial.print(" y2:");
    // Serial.print(y2);



    // Serial.println();
}


//=============================================================================
void loop() {


    taskIMU();
    taskSerialOut();
    taskTFT();

    delay(LOOP_DT); // Read at 100Hz
}
